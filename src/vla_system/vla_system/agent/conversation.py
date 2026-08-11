"""Conversation memory and the situation payload handed to the model.

The history is the state machine. What used to be a ``target_queue`` and an
``awaiting_clarification`` flag is now just the fact that the model can read
back what it decided two turns ago.

What is *not* in the history: scene snapshots and robot state. Those are
rebuilt fresh on every call and attached to the newest message only. Letting
them accumulate would both blow up the context and, worse, leave the model
reasoning over positions that were true thirty seconds ago.

Persistence
-----------
``_items`` is a bounded window -- only what the model still sees. If a
``log_path`` is given, every item is also appended to that file before
trimming, so the full transcript survives ``_trim`` dropping old turns from
memory, and survives the node restarting.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path


def is_pickable(scene_object) -> bool:
    """Can the executor act on this object at all?

    Always true, and the reason matters: cobot2_ws's pick_fsm computes the
    grasp coordinate on its own side from the object's class name alone
    (bridge/pick_bridge.py -- only ``class`` crosses, never a base-frame
    position), so this ws's own table calibration is not a precondition for
    picking. ``position_valid`` used to gate this and no longer does.

    It is a named function rather than a literal because **two** layers ask
    the question now -- the conversational path through ``scene_to_payload``
    below, and the rule layer through ``AgentNode.scene_items``. They were
    briefly out of step (2026-08-11, merge): the rules still read
    ``position_valid`` while the prompt was told everything is pickable, so
    Tier 1 silently found no candidates and answered "담을 게 없습니다" while
    Tier 2 picked the same object happily. Nothing raised. One function is
    what stops that shape of bug from coming back.
    """
    del scene_object          # kept in the signature: the answer may narrow again
    return True


def scene_to_payload(scene, max_objects: int = 40) -> dict:
    """Flatten a SceneSnapshot into the JSON the model reads.

    ``position_base`` is still included when this ws's perception happened to
    resolve one, purely as operator-facing context (shown in vla_gui's debug
    panel) -- the model does not gate on it.
    """
    if scene is None:
        return {"visible_objects": [], "note": "아직 카메라 장면을 받지 못했습니다."}

    objects = []
    for scene_object in list(scene.objects)[:max_objects]:
        entry = {
            "id": scene_object.id,
            "class": scene_object.class_name,
            "pickable": is_pickable(scene_object),
        }
        if scene_object.color and scene_object.color != "unknown":
            entry["color"] = scene_object.color
        if scene_object.position_valid:
            entry["position_base"] = [
                round(float(scene_object.position_base.x), 3),
                round(float(scene_object.position_base.y), 3),
                round(float(scene_object.position_base.z), 3),
            ]
        objects.append(entry)

    return {"visible_objects": objects}


def robot_state_to_payload(state) -> dict:
    """The arm's ground truth. Overrides whatever the model remembers."""
    if state is None:
        return {"status": "unknown", "holding": None}

    payload = {
        "status": state.status or "unknown",
        "holding": (
            {
                "id": state.holding_object_id,
                "class": state.holding_class_name,
            }
            if state.holding_object_id
            else None
        ),
        "motion_enabled": bool(state.motion_enabled),
    }
    if state.current_action:
        payload["current_action"] = {
            "name": state.current_action,
            "action_id": state.current_action_id,
        }
    if state.last_action:
        payload["last_action"] = {
            "name": state.last_action,
            "result": state.last_result,
            "details": state.details,
        }
    return payload


def build_situation(event: dict, scene_payload: dict, state_payload: dict) -> str:
    return json.dumps(
        {"event": event, "scene": scene_payload, "robot_state": state_payload},
        ensure_ascii=False,
    )


class Conversation:
    """Bounded history in Responses-API item form."""

    def __init__(self, max_items: int = 60, log_path: str | Path | None = None):
        if max_items < 4:
            raise ValueError("max_items must leave room for at least one turn")
        self.max_items = max_items
        self._items: list[dict] = []

        # log_path stays set after close() so a caller that decides not to
        # keep this session's log (the eval harness, for one) can still find
        # the file to remove it.
        self.log_path: Path | None = None
        self._log_file = None
        if log_path:
            self.log_path = Path(log_path)
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_file = self.log_path.open("a", encoding="utf-8")

    def _persist(self, item: dict) -> None:
        if self._log_file is None:
            return
        record = {"logged_at": datetime.now(timezone.utc).isoformat(), **item}
        self._log_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._log_file.flush()

    def close(self) -> None:
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    # ------------------------------------------------------------- mutation

    def add_user(self, content: str) -> None:
        item = {"role": "user", "content": content}
        self._items.append(item)
        self._persist(item)
        self._trim()

    def add_assistant(self, content: str) -> None:
        if not content.strip():
            return
        item = {"role": "assistant", "content": content}
        self._items.append(item)
        self._persist(item)
        self._trim()

    def add_function_call(self, call_id: str, name: str, arguments: str) -> None:
        item = {
            "type": "function_call",
            "call_id": call_id,
            "name": name,
            "arguments": arguments,
        }
        self._items.append(item)
        self._persist(item)
        self._trim()

    def add_function_output(self, call_id: str, output: str) -> None:
        item = {"type": "function_call_output", "call_id": call_id, "output": output}
        self._items.append(item)
        self._persist(item)
        self._trim()

    def clear(self) -> None:
        self._items.clear()

    # ---------------------------------------------------------------- reads

    def items(self) -> list[dict]:
        return list(self._items)

    def __len__(self) -> int:
        return len(self._items)

    # --------------------------------------------------------------- window

    def _trim(self) -> None:
        """Drop whole turns off the front, never half of one.

        Cutting at an arbitrary index is what breaks these transcripts: a
        ``function_call_output`` whose matching ``function_call`` fell off the
        front is rejected by the API outright. Turn boundaries -- user messages
        -- are the only safe cut points.
        """
        if len(self._items) <= self.max_items:
            return

        target = len(self._items) - self.max_items
        for index in range(target, len(self._items)):
            item = self._items[index]
            if item.get("role") == "user":
                self._items = self._items[index:]
                return
        # No user message left to cut at: the tail is one very long tool run.
        # Dropping it whole is better than emitting an orphaned output.
        self._items = []

"""Pure JSON-boundary logic for talking to cobot2_ws's ``vla_command_node``.

Everything here is plain dicts, strings and dataclasses on purpose: the
authoritative contract lives in a *different git clone*
(``~/cobot2_ws/md/vla-bridge-contract.md``), not copied into this repo
(CLAUDE.md #2 -- a second copy is how the two drift). Keeping the JSON shape
and the result-mapping table in pure functions means this file can be tested
without a live cobot2_ws process, and it is the one place to re-read that
contract against if the two ever disagree.

Boundary recap (2026-08-10): only ``class`` (never ``object_id``) crosses to
cobot2_ws, and cobot2_ws owns the actual grasp coordinates now -- this ws's
job is picking *which* class, not *where*.
"""

from dataclasses import dataclass
import json

# agent/tools.py's MOTION_TOOLS is pick_and_place only now -- pick_and_hold
# and release were removed there (cobot2_ws's pick_fsm always carries a pick
# through to place, no "hold it and wait" or "put it down right here" on that
# side, vla-bridge-contract.md #7), so this bridge no longer needs to reject
# them; the model cannot produce them in the first place.

# cobot2_ws's parse_command() treats "pick" and "pick_and_place" as synonyms;
# using pick_and_place here keeps the JSON cmd matching the RobotAction.name
# it came from.
PICK_CMD = "pick_and_place"
ABORT_CMD = "abort"

# vla-bridge-contract.md #5's PLACE_VALUES -- must stay identical to
# cobot2_ws's voice_processing/vla_command_node.py PLACE_VALUES (no shared
# import across the two clones; both sides hand-copy this set, same pattern
# that file uses for the same reason).
PLACE_VALUES = frozenset({"basket", "table", "discard"})

# table/discard joint poses are copied placeholders (home joints), not
# re-taught to a safe place yet (contract #5, confirmed 2026-08-10 by
# cobot2_ws). Sending them for real motion is the one thing the contract
# explicitly asks this side not to do until that changes.
UNVERIFIED_PLACE_VALUES = frozenset({"table", "discard"})

# vla-bridge-contract.md #3 + vla-integration.md #3-3's own mapping table:
# how one /vla/pick_result.result maps onto RobotState.last_result.
# "superseded" -> "rejected" is that table's choice, not a guess made here.
_RESULT_TO_LAST_RESULT = {
    "succeeded": "succeeded",
    "failed": "failed",
    "rejected": "rejected",
    "superseded": "rejected",
}


def build_pick_command(
    *,
    class_name: str,
    place: str,
    request_id: str,
    stamp_ns: int,
    pixel: tuple[float, float] | None = None,
    pixel_wh: tuple[int, int] | None = None,
) -> dict:
    """``/vla/pick_command`` payload for one pick_and_place action.

    ``place`` is always sent now that ``RobotAction``/``tools.py`` carry it
    (contract #2/#5/#7) -- the caller (``vla_pick_bridge_node``) is
    responsible for having already rejected an empty or unverified value via
    :func:`place_rejection_reason` before this is called; this function does
    not re-check, it just puts the string on the wire.

    ``pixel``/``pixel_wh`` are optional (contract #2): omitted when the scene
    hasn't reported a frame resolution yet (``SceneSnapshot.image_width/
    height`` still 0 -- see ``bbox_center``). cobot2_ws validates but ignores
    both today (``pixel_policy=warn``, contract #8, no ``select_by_point()``
    yet) -- sent anyway so the field is populated the day that lands there,
    instead of a second round of "add the field" on this side.
    """
    payload = {
        "cmd": PICK_CMD,
        "class": class_name,
        "place": place,
        "request_id": request_id,
        "stamp_ns": stamp_ns,
    }
    if pixel is not None and pixel_wh is not None:
        payload["pixel"] = list(pixel)
        payload["pixel_wh"] = list(pixel_wh)
    return payload


def place_rejection_reason(place: str, *, allow_unverified: bool) -> str | None:
    """``None`` means ``place`` is fine to send onward. Otherwise, why not.

    Two independent failure modes, checked in order: an unknown value (typo,
    or a future cobot2_ws vocabulary this side hasn't caught up to), and a
    known-but-unverified value gated behind ``allow_unverified_place``
    (contract #5 -- table/discard are placeholder joint poses on real
    hardware as of 2026-08-10, not something to send without a flag someone
    flipped on purpose after teaching them).
    """
    if place not in PLACE_VALUES:
        return f"place 값이 올바르지 않습니다: {place!r} (허용: {sorted(PLACE_VALUES)})"
    if place in UNVERIFIED_PLACE_VALUES and not allow_unverified:
        return (
            f"'{place}'는 아직 실기에서 검증되지 않은 위치라 사용할 수 없습니다 "
            "(teach 완료 전까지 basket만 허용 -- vla-bridge-contract.md #5)"
        )
    return None


def build_abort_command(*, request_id: str, reason: str, stamp_ns: int) -> dict:
    """``/vla/pick_command`` payload that maps to cobot2_ws's ``/pick/abort``."""
    return {
        "cmd": ABORT_CMD,
        "request_id": request_id,
        "reason": reason,
        "stamp_ns": stamp_ns,
    }


def find_scene_object(scene_objects, object_id: str):
    """The one lookup this bridge exists to do: ``object_id`` -> the object.

    Returns the ``SceneObject`` itself (not just ``class_name``) because the
    pixel center used for ``select_by_point()`` (contract #8) comes from its
    bbox -- the caller pulls whatever fields it needs off the result.
    """
    for scene_object in scene_objects:
        if scene_object.id == object_id:
            return scene_object
    return None


def find_class_name(scene_objects, object_id: str) -> str | None:
    """``object_id`` -> ``class``. Only ``class`` crosses as the class
    filter; an individual-object handle like ``apple_17`` means nothing to
    cobot2_ws's FSM on its own -- pairing it with a pixel is what
    :func:`bbox_center` is for.
    """
    scene_object = find_scene_object(scene_objects, object_id)
    return scene_object.class_name if scene_object is not None else None


def bbox_center(scene_object) -> tuple[float, float]:
    """Pixel center of a ``SceneObject``'s bbox, for the ``pixel`` field.

    The bbox is already in the same frame ``vla_perception`` subscribed
    (``image_topic``) -- the same physical D435i cobot2_ws's own
    segmentation runs on now that the camera is confirmed shared
    (2026-08-10/11), so this pixel needs no reprojection to mean something
    on cobot2_ws's side. Center, not mask centroid: this bridge only ever
    sees the bbox (``SceneObject.x_min/y_min/x_max/y_max``), never the mask
    itself -- that stays inside ``vla_perception``.
    """
    return (
        (scene_object.x_min + scene_object.x_max) / 2.0,
        (scene_object.y_min + scene_object.y_max) / 2.0,
    )


# --- Live FSM step visibility (docs/context/fsm-state-integration.md) --------
#
# cobot2_ws's pick_fsm already publishes /pick/state (std_msgs/String, bare
# State enum name, every transition -- task_manager.py). This maps those names
# onto RobotState so the GUI can show "지금 어느 단계인지". The name set below
# mirrors pick_fsm/states.py's State enum; cobot2_ws owns the source (no shared
# import across the two clones -- same hand-copy pattern as PLACE_VALUES).

# States where the gripper physically holds the object (states.py HOLDING_STATES).
FSM_HOLDING_STATES = frozenset({"VERIFY", "LIFT", "PLACE", "PLACE_RETRY"})

# State name -> (RobotState.status, human label for RobotState.details).
# Unlisted names fall through to ("moving", the raw name).
_FSM_STATE_INFO = {
    "IDLE": ("idle", "대기"),
    "LISTENING": ("moving", "지시 듣는 중"),
    "PERCEIVE": ("moving", "물체 인식 중"),
    "SCENE_PREP": ("moving", "충돌 물체 등록 중"),
    "PLAN": ("moving", "파지 계획 중"),
    "NEXT_CANDIDATE": ("moving", "다음 후보 검토 중"),
    "WAIT_APPROVAL": ("waiting_approval", "사람 승인 대기"),
    "STOW": ("moving", "그리퍼 정리 중"),
    "APPROACH": ("moving", "접근 중"),
    "OPEN_GRIPPER": ("moving", "그리퍼 여는 중"),
    "DESCEND": ("moving", "하강 중"),
    "CLOSE": ("moving", "그리퍼 닫는 중"),
    "VERIFY": ("holding", "파지 확인 중"),
    "RELEASE_RETRY": ("moving", "놓치고 재시도 중"),
    "LIFT": ("holding", "들어올리는 중"),
    "PLACE": ("holding", "놓는 자세로 이동 중"),
    "PLACE_RETRY": ("holding", "놓기 재시도 대기"),
    "RELEASE": ("moving", "놓는 중"),
    "HOME": ("moving", "홈 복귀 중"),
    "SPEAK_FAIL": ("error", "실패 통보"),
    "ABORT": ("moving", "중단 중"),
    "SAFE_STOP": ("error", "정지 유지 (리셋 대기)"),
}


@dataclass(frozen=True)
class FsmStateView:
    status: str  # RobotState.status
    label: str  # RobotState.details, human-readable step
    holding: bool  # gripper holds the object in this state


def fsm_state_view(state_name: str) -> FsmStateView:
    """One /pick/state value -> how it should look in RobotState."""
    status, label = _FSM_STATE_INFO.get(state_name, ("moving", state_name or "동작 중"))
    return FsmStateView(status=status, label=label, holding=state_name in FSM_HOLDING_STATES)


@dataclass(frozen=True)
class ResultUpdate:
    terminal: bool  # False while still "accepted" (handed off, in progress)
    status: str  # RobotState.status
    last_result: str  # "" while not terminal


def parse_pick_result(raw: str) -> dict | None:
    """Best-effort JSON decode. ``None`` means "could not even parse it."."""
    try:
        doc = json.loads(raw)
    except (ValueError, TypeError):
        return None
    return doc if isinstance(doc, dict) else None


def result_update(result: str) -> ResultUpdate:
    """What one ``/vla/pick_result.result`` value means for our RobotState.

    ``"accepted"`` is not terminal: ``current_action`` stays set and
    ``last_result`` stays empty so ``agent_node``'s own duplicate-suppression
    (``if not message.last_result or message.current_action: return``) keeps
    waiting instead of treating the handoff itself as a decision point.
    """
    if result == "accepted":
        return ResultUpdate(terminal=False, status="moving", last_result="")
    last_result = _RESULT_TO_LAST_RESULT.get(result)
    if last_result is None:
        # Not a value the contract defines. Treated as a failure rather than
        # silently ignored -- a request must not vanish without ever
        # releasing the "one action in flight" gate.
        return ResultUpdate(terminal=True, status="idle", last_result="failed")
    return ResultUpdate(terminal=True, status="idle", last_result=last_result)

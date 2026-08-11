#!/usr/bin/env python3
"""The judgement layer. Everything the old rule engine did happens here.

The model is called at *decision points*, never per frame:

  - the user said something
  - an action the model started has finished, failed, or was cancelled
  - the emergency stop fired, so the model needs to know the world changed

Each call gets the conversation so far plus a freshly built snapshot of what
the camera sees and what the arm is really doing. What comes back is a tool
call, which is turned into a RobotAction, a question for the user, or nothing.

The LLM round-trip happens on a worker thread. Blocking the executor would
stall the scene and robot-state subscriptions the *next* decision depends on,
and would make the emergency stop event queue up behind an API call -- which
is exactly the thing the stop path exists to avoid.
"""

import json
import queue
import threading
import uuid
from datetime import datetime
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vla_interfaces.msg import AgentReply, RobotAction, RobotState, SceneSnapshot

from vla_system.agent.conversation import (
    Conversation,
    build_situation,
    robot_state_to_payload,
    scene_to_payload,
)
from vla_system.agent.llm import AgentLLM
from vla_system.agent.rules import RuleStore
from vla_system.agent.skill_tier import SceneItem, SkillTier, make_parser
from vla_system.agent.tools import MOTION_TOOLS
from vla_system.agent.vision import encode_frame


class AgentNode(Node):
    def __init__(self, **node_kwargs):
        # node_kwargs exists for parameter_overrides. Some parameters decide
        # what gets built here in __init__ -- vision_enabled decides whether
        # the camera topic is subscribed at all -- and setting those after
        # construction is too late. Launch delivers them the same way.
        super().__init__("vla_agent", **node_kwargs)

        self.declare_parameter("utterance_topic", "/vla/user_utterance")
        self.declare_parameter("scene_topic", "/vla/scene")
        self.declare_parameter("robot_state_topic", "/vla/robot/state")
        self.declare_parameter("action_topic", "/vla/robot/action")
        self.declare_parameter("stop_topic", "/vla/robot/stop")
        self.declare_parameter("estop_topic", "/vla/estop")
        self.declare_parameter("reply_topic", "/vla/agent/reply")
        self.declare_parameter("annotated_topic", "/vla/perception/annotated_image")

        self.declare_parameter("model", "gpt-5-mini")
        self.declare_parameter("stt_model", "gpt-4o-transcribe")
        self.declare_parameter("env_file", "")
        self.declare_parameter("request_timeout_s", 30.0)
        self.declare_parameter("max_tool_rounds", 4)
        self.declare_parameter("max_history_items", 60)
        self.declare_parameter("max_consecutive_failures", 3)
        self.declare_parameter("continue_after_action", True)
        # Empty string turns logging off. Independent of max_history_items:
        # that window bounds what the LLM sees, this bounds nothing -- the
        # full transcript survives trimming and the node restarting.
        self.declare_parameter("conversation_log_dir", "~/.ros/vla_conversations")
        # Tier 1 -- the rule layer in front of this node. On by default since
        # 2026-08-11: it was off while the layer was being proven, and the
        # measurements are in (md/A4_INTEGRATION.md §7). Turning it off still
        # restores the previous behaviour exactly -- every utterance goes to
        # the LLM and nothing is remembered between sessions -- so the rollback
        # is this parameter, not a revert. Off is not the safer setting: the
        # layer adds no motion of its own, and it is what refuses to pick up
        # scissors without asking.
        self.declare_parameter("skill_tier_enabled", True)
        # Long-term rules outlive the process. Empty string keeps them in
        # memory only, which is what the evaluation harness wants.
        self.declare_parameter("rule_store_path", "~/.ros/vla_rules.json")
        # The labelled camera frame, sent with what the user said. Without it
        # "이거 집어줘" cannot be answered at all -- see agent/vision.py. Off
        # makes the agent text-only again, exactly as it was before.
        self.declare_parameter("vision_enabled", True)
        self.declare_parameter("vision_max_width", 640)
        self.declare_parameter("vision_jpeg_quality", 70)
        # A frame older than this is not sent. A dead camera would otherwise
        # keep answering "이거" with a picture of a table that has since been
        # cleared -- worse than having no picture, because it looks answered.
        self.declare_parameter("vision_max_age_s", 2.0)

        self.max_tool_rounds = int(self.get_parameter("max_tool_rounds").value)
        self.max_consecutive_failures = int(
            self.get_parameter("max_consecutive_failures").value
        )
        self.continue_after_action = bool(
            self.get_parameter("continue_after_action").value
        )
        self.conversation = Conversation(
            max_items=int(self.get_parameter("max_history_items").value),
            log_path=self._conversation_log_path(),
        )

        self.skill_tier: SkillTier | None = None
        self._skill_pending = False   # Tier 1 asked something and is waiting

        self.scene: SceneSnapshot | None = None
        self.robot_state: RobotState | None = None
        self.lock = threading.Lock()

        # Robot state is published several times per action (moving, grasped,
        # finished). Only the transition into a *result* is a decision point.
        self.seen_result_key: tuple | None = None
        self.consecutive_failures = 0
        # A stop produces two things the agent hears: the estop event itself
        # and, a moment later, the cancelled action's result. Both describe one
        # occurrence, so the second is folded into the first rather than
        # costing a second API round-trip.
        self.expect_cancel_result = False
        # A stop has to invalidate the decision in flight, not just the motion
        # in flight. An LLM round-trip takes seconds; a stop landing inside
        # that window would otherwise be followed by an action built from the
        # pre-stop world, and the arm would move *after* the user said stop.
        self.stop_epoch = 0
        self.turn_epoch = 0
        self.turn_stamp = None
        self.turn_spoke = False

        # Latest labelled frame, kept as BGR. Encoding to JPEG happens on the
        # worker thread at decision time, not here -- this callback runs at
        # camera rate on the executor thread, and that thread also carries the
        # scene and stop subscriptions.
        self.frame = None
        self.frame_monotonic = 0.0

        self.llm: AgentLLM | None = None
        self.llm_error = ""
        self.events: queue.Queue = queue.Queue()
        self.shutdown = threading.Event()

        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.action_publisher = self.create_publisher(
            RobotAction, str(self.get_parameter("action_topic").value), command_qos
        )
        self.stop_publisher = self.create_publisher(
            String, str(self.get_parameter("stop_topic").value), command_qos
        )
        self.reply_publisher = self.create_publisher(
            AgentReply, str(self.get_parameter("reply_topic").value), command_qos
        )

        self.create_subscription(
            String,
            str(self.get_parameter("utterance_topic").value),
            self.utterance_callback,
            command_qos,
        )
        self.create_subscription(
            SceneSnapshot,
            str(self.get_parameter("scene_topic").value),
            self.scene_callback,
            stream_qos,
        )
        self.create_subscription(
            RobotState,
            str(self.get_parameter("robot_state_topic").value),
            self.robot_state_callback,
            latched_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("estop_topic").value),
            self.estop_callback,
            command_qos,
        )
        if bool(self.get_parameter("vision_enabled").value):
            # Same QoS as the scene stream: perception publishes both from the
            # same frame, and RELIABLE here would simply never match a
            # BEST_EFFORT publisher -- the subscription would sit silent with
            # no error at all.
            self.create_subscription(
                Image,
                str(self.get_parameter("annotated_topic").value),
                self.annotated_callback,
                stream_qos,
            )

        self.worker = threading.Thread(
            target=self.run_worker, name="vla-agent-worker", daemon=True
        )
        self.worker.start()
        self.get_logger().info(
            f"agent ready: model={self.get_parameter('model').value} "
            f"tool_rounds={self.max_tool_rounds}"
        )

    # ------------------------------------------------------------ callbacks

    def scene_callback(self, message: SceneSnapshot) -> None:
        with self.lock:
            self.scene = message

    def annotated_callback(self, message: Image) -> None:
        from vla_system.perception.detector import image_message_to_bgr

        try:
            frame = image_message_to_bgr(message)
        except Exception as exc:                       # noqa: BLE001
            # A frame we cannot decode must not take the last good one down
            # with it, and must not spam the log at camera rate either.
            self.get_logger().warning(
                f"주석 프레임을 읽지 못했습니다: {exc}", throttle_duration_sec=10.0
            )
            return
        with self.lock:
            self.frame = frame
            self.frame_monotonic = monotonic()

    def current_frame_image(self) -> str:
        """이번 판단에 실어 보낼 사진. 없거나 오래됐으면 빈 문자열."""
        if not bool(self.get_parameter("vision_enabled").value):
            return ""
        with self.lock:
            frame, taken_at = self.frame, self.frame_monotonic
        if frame is None:
            return ""

        age = monotonic() - taken_at
        max_age = float(self.get_parameter("vision_max_age_s").value)
        if max_age > 0 and age > max_age:
            # Stale beyond use. Saying nothing is right: the model is told the
            # picture may be absent, and a picture of a table that has since
            # changed would be answered confidently and wrongly.
            self.get_logger().warning(
                f"카메라 화면이 {age:.1f}초 지나 이번 판단에서 뺐습니다",
                throttle_duration_sec=10.0,
            )
            return ""
        return encode_frame(
            frame,
            max_width=int(self.get_parameter("vision_max_width").value),
            quality=int(self.get_parameter("vision_jpeg_quality").value),
        )

    def utterance_callback(self, message: String) -> None:
        text = message.data.strip()
        if not text:
            return
        self.consecutive_failures = 0
        self.events.put({"type": "user_said", "text": text})

    def estop_callback(self, message: String) -> None:
        reason = message.data.strip() or "정지"
        self.expect_cancel_result = True
        self.stop_epoch += 1
        self.events.put(
            {
                "type": "emergency_stop",
                "detail": (
                    f"사용자가 '{reason}'이라고 해서 코드가 LLM을 거치지 않고 "
                    "로봇을 즉시 멈췄습니다."
                ),
            }
        )

    def robot_state_callback(self, message: RobotState) -> None:
        with self.lock:
            self.robot_state = message

        if not message.last_result or message.current_action:
            return
        key = (message.last_action_id, message.last_action, message.last_result)
        if key == self.seen_result_key:
            return
        self.seen_result_key = key

        if message.last_result == "succeeded":
            self.consecutive_failures = 0
        elif message.last_result != "cancelled":
            # A cancel is the user getting what they asked for, not a fault.
            # Counting it would let three deliberate stops mute the agent.
            self.consecutive_failures += 1

        if message.last_result == "cancelled" and self.expect_cancel_result:
            self.expect_cancel_result = False
            return

        if not self.continue_after_action:
            return
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.publish_reply(
                "error",
                f"동작이 {self.consecutive_failures}번 연속 실패해서 자동 진행을 "
                "멈췄습니다. 어떻게 할지 말씀해 주세요.",
            )
            return

        self.events.put(
            {
                "type": "action_finished",
                "action": message.last_action,
                "result": message.last_result,
                "detail": message.details,
            }
        )

    # -------------------------------------------------------------- outputs

    def publish_reply(self, kind: str, text: str, object_ids=None) -> None:
        message = AgentReply()
        message.header.stamp = self.get_clock().now().to_msg()
        message.kind = kind
        message.text = text
        message.focus_object_ids = list(object_ids or [])
        self.reply_publisher.publish(message)

    def publish_action(
        self, name: str, object_id: str, reason: str, place: str = ""
    ) -> str:
        message = RobotAction()
        message.header.stamp = self.turn_stamp or self.get_clock().now().to_msg()
        message.action_id = uuid.uuid4().hex[:12]
        message.name = name
        message.object_id = object_id
        message.place = place
        message.reason = reason
        self.action_publisher.publish(message)
        self.get_logger().info(
            f"action {name}({object_id}, place={place!r}) id={message.action_id}"
        )
        return message.action_id

    def publish_stop(self, reason: str) -> None:
        self.stop_publisher.publish(String(data=reason))

    # --------------------------------------------------------------- worker

    def run_worker(self) -> None:
        while not self.shutdown.is_set():
            try:
                event = self.events.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.route(event)
            except Exception as exc:  # never let the loop die on one bad turn
                self.get_logger().error(f"decision failed: {exc}")
                self.publish_reply("error", f"판단 중 오류가 발생했습니다: {exc}")

    def get_llm(self) -> AgentLLM:
        if self.llm is None:
            env_file = str(self.get_parameter("env_file").value) or None
            self.llm = AgentLLM(
                model=str(self.get_parameter("model").value),
                stt_model=str(self.get_parameter("stt_model").value),
                env_file=env_file,
                timeout_s=float(self.get_parameter("request_timeout_s").value),
            )
        return self.llm

    # ------------------------------------------------------------- storage

    def _conversation_log_path(self) -> Path | None:
        """One file per node start. Timestamp is the only identity it needs --
        nothing else in this node distinguishes one run from the next."""
        raw = str(self.get_parameter("conversation_log_dir").value)
        if not raw:
            return None
        log_dir = Path(raw).expanduser()
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        return log_dir / f"{stamp}.jsonl"

    # ------------------------------------------------------------- Tier 1

    def get_skill_tier(self) -> SkillTier | None:
        """Built on first use, like the LLM client -- constructing it needs the
        same OpenAI client, and a node that never gets an utterance should not
        require an API key to start."""
        if not bool(self.get_parameter("skill_tier_enabled").value):
            return None
        if self.skill_tier is None:
            raw = str(self.get_parameter("rule_store_path").value)
            store = RuleStore(Path(raw).expanduser() if raw else None)
            parse = make_parser(self.get_llm().client,
                                str(self.get_parameter("model").value))
            self.skill_tier = SkillTier(self, store, parse)
            self.get_logger().info(f"skill tier on, rules at {raw or '(memory)'}")
        return self.skill_tier

    # SkillHost. The rule layer speaks and acts through the same publishers the
    # conversational path uses, so nothing downstream can tell them apart.

    def scene_items(self) -> list[SceneItem]:
        with self.lock:
            scene = self.scene
        if scene is None:
            return []
        return [
            SceneItem(
                object_id=o.id,
                class_name=o.class_name,
                color=o.color,
                # No depth fix means the executor cannot reach it, whatever the
                # rules decide. Same gate the conversational path applies.
                pickable=bool(o.position_valid),
                rank=int(o.track_id),
            )
            for o in scene.objects
        ]

    def pick(self, object_id: str, reason: str) -> None:
        # place stays empty: the rule layer has no slot for a destination, so
        # the bridge default (basket) applies. Utterances that name a place go
        # to the conversational path, which does have that slot.
        self.publish_action("pick_and_place", object_id, reason, "")

    def escalate(self, reason: str, text: str, mission_text: str) -> None:
        """Tier 1 declined. Hand the turn to the conversational agent."""
        self._skill_pending = False
        self.get_logger().info(f"skill tier -> llm ({reason})")
        if mission_text:
            # Without this the LLM does not know what was already underway and
            # answers the correction as if it were a fresh request.
            self.conversation.add_user(json.dumps({
                "event": "skill_escalation", "reason": reason,
                "original_utterance": mission_text,
            }, ensure_ascii=False))
        remembered = self.skill_tier.store.describe_all() if self.skill_tier else ""
        if remembered and self.skill_tier and self.skill_tier.store.all:
            self.conversation.add_user(json.dumps({
                "event": "remembered_rules", "detail": remembered,
            }, ensure_ascii=False))
        self.decide({"type": "user_said", "text": text})

    def note(self, event: dict) -> None:
        self.conversation.add_user(json.dumps(event, ensure_ascii=False))

    def record(self, tier: str, detail: str = "") -> None:
        """Instrumentation hook. The harness overrides it; here it only logs."""
        if detail:
            self.get_logger().debug(f"tier={tier} {detail}")

    def turn_done(self) -> None:
        self._skill_pending = False

    def say(self, text: str) -> None:
        self.publish_reply("say", text)

    def ask(self, text: str) -> None:
        self._skill_pending = True
        self.publish_reply("ask_clarification", text)

    # ------------------------------------------------------------- decision

    def route(self, event: dict) -> None:
        """Try Tier 1 first; fall through to the conversational agent.

        Only user utterances and the completion of a rule-dispatched action go
        to Tier 1. Everything else -- errors, stops, state the rules have no
        opinion on -- belongs to the conversation.
        """
        tier = self.get_skill_tier()
        if tier is None:
            self.decide(event)
            return
        if event.get("type") == "action_finished" and tier.busy:
            tier.on_action_finished()
            return
        if event.get("type") == "user_said":
            self.turn_epoch = self.stop_epoch
            self.turn_stamp = self.get_clock().now().to_msg()
            tier.handle(event["text"])
            return
        self.decide(event)

    def decide(self, event: dict) -> None:
        # Stamped once, at the point the world was read -- not at publish time.
        # The executor compares this against when it last stopped, so it can
        # tell "decided before the stop" from "decided after it" no matter how
        # long the model took to answer.
        self.turn_epoch = self.stop_epoch
        self.turn_stamp = self.get_clock().now().to_msg()

        with self.lock:
            scene, state = self.scene, self.robot_state

        self.conversation.add_user(
            build_situation(event, scene_to_payload(scene), robot_state_to_payload(state))
        )

        # Only what a person said can be ambiguous in a way a picture settles.
        # "이거" needs the frame; "the action you started finished" does not,
        # and sending one there would pay for vision on every step of a
        # multi-object mission for nothing.
        image = self.current_frame_image() if event.get("type") == "user_said" else ""

        try:
            llm = self.get_llm()
        except Exception as exc:
            self.publish_reply("error", f"LLM을 초기화하지 못했습니다: {exc}")
            return

        for _ in range(self.max_tool_rounds):
            try:
                response = llm.respond(self.conversation.items(), image=image)
            except Exception as exc:
                self.publish_reply("error", f"LLM 호출에 실패했습니다: {exc}")
                return
            # Rounds after the first are the model working through tool
            # results, not re-reading the table. The reference it needed the
            # picture for is already resolved into a call by now.
            image = ""

            # Free text and the tool's own `say` are two routes to the same
            # place, so only one of them is spoken per round.
            self.turn_spoke = bool(response.text)
            if response.text:
                self.conversation.add_assistant(response.text)
                self.publish_reply("say", response.text)

            if not response.calls:
                return

            ends_turn = False
            for call in response.calls:
                self.conversation.add_function_call(
                    call.call_id, call.name, call.raw_arguments
                )
                output, call_ends_turn = self.dispatch(call)
                self.conversation.add_function_output(
                    call.call_id, json.dumps({"result": output}, ensure_ascii=False)
                )
                ends_turn = ends_turn or call_ends_turn
            if ends_turn:
                return

        self.publish_reply(
            "error", "판단이 정리되지 않아 이번 턴을 중단했습니다. 다시 말씀해 주세요."
        )

    # ------------------------------------------------------------- dispatch

    def find_object(self, object_id: str):
        with self.lock:
            scene = self.scene
        if scene is None:
            return None
        for scene_object in scene.objects:
            if scene_object.id == object_id:
                return scene_object
        return None

    def speak(self, call) -> str:
        """Say the tool's sentence, unless this round already said something."""
        sentence = str(call.arguments.get("say", "")).strip()
        if sentence and not self.turn_spoke:
            self.turn_spoke = True
            self.conversation.add_assistant(sentence)
            self.publish_reply("say", sentence)
        return sentence

    def dispatch(self, call) -> tuple[str, bool]:
        """Return (what the model is told, whether the turn ends here)."""
        if call.parse_error:
            return call.parse_error, False

        name = call.name

        if name in MOTION_TOOLS:
            if self.stop_epoch != self.turn_epoch:
                self.get_logger().warning(
                    f"withheld {name}: a stop landed during this decision "
                    f"(epoch {self.turn_epoch} -> {self.stop_epoch})"
                )
                return (
                    "이 판단을 시작한 뒤에 정지가 걸렸습니다. 동작을 보내지 않았습니다. "
                    "곧 최신 상태로 다시 판단할 기회가 주어집니다.",
                    True,
                )
            reason = self.speak(call)

            if name == "release":
                self.publish_action("release", "", reason)
                return "들고 있던 물체를 놓는 중입니다. 완료되면 알려드리겠습니다.", True

            object_id = str(call.arguments.get("object_id", "")).strip()
            # Checked here rather than at the arm: a wrong id caught now costs
            # the model one extra tool round, whereas letting it through costs
            # a full motion attempt and a failure event.
            scene_object = self.find_object(object_id)
            if scene_object is None:
                return (
                    f"'{object_id}'는 지금 화면에 없습니다. scene.visible_objects에 "
                    "있는 id 중에서 다시 고르세요.",
                    False,
                )
            if not scene_object.position_valid:
                return (
                    f"'{object_id}'는 아직 3D 위치를 확정하지 못해 집을 수 없습니다.",
                    False,
                )
            # Only pick_and_place carries a destination -- pick_and_hold has no
            # `place` in its schema (tools.py), so this is "" for that tool.
            place = str(call.arguments.get("place", "")).strip()
            self.publish_action(name, object_id, reason, place)
            return f"{object_id} 동작을 시작했습니다. 완료되면 알려드리겠습니다.", True

        if name == "cancel_current_action":
            self.speak(call)
            self.publish_stop("agent: cancel_current_action")
            # The turn ends here on purpose. Issuing a new motion in the same
            # round would race the stop: the arm could still be reporting busy
            # and reject it, or worse, accept it before braking.
            return (
                "진행 중이던 동작을 중단했습니다. 중단이 반영되면 다시 판단 기회가 "
                "주어집니다.",
                True,
            )

        if name == "ask_clarification":
            question = str(call.arguments.get("question", "")).strip()
            object_ids = [
                str(value).strip()
                for value in call.arguments.get("object_ids", []) or []
                if str(value).strip()
            ]
            if not question:
                return "question이 비어 있습니다. 물어볼 문장을 채워서 다시 호출하세요.", False
            self.publish_reply("ask_clarification", question, object_ids)
            return "사용자에게 물었습니다. 답을 기다리세요.", True

        if name == "wait":
            self.speak(call)
            return "대기합니다.", True

        return f"'{name}'은(는) 사용할 수 없는 함수입니다.", False

    def close(self) -> None:
        self.shutdown.set()
        self.worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

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

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vla_interfaces.msg import AgentReply, RobotAction, RobotState, SceneSnapshot

from vla_system.agent.conversation import (
    Conversation,
    build_situation,
    robot_state_to_payload,
    scene_to_payload,
)
from vla_system.agent.llm import AgentLLM
from vla_system.agent.tools import MOTION_TOOLS


class AgentNode(Node):
    def __init__(self):
        super().__init__("vla_agent")

        self.declare_parameter("utterance_topic", "/vla/user_utterance")
        self.declare_parameter("scene_topic", "/vla/scene")
        self.declare_parameter("robot_state_topic", "/vla/robot/state")
        self.declare_parameter("action_topic", "/vla/robot/action")
        self.declare_parameter("stop_topic", "/vla/robot/stop")
        self.declare_parameter("estop_topic", "/vla/estop")
        self.declare_parameter("reply_topic", "/vla/agent/reply")

        self.declare_parameter("model", "gpt-5-mini")
        self.declare_parameter("stt_model", "gpt-4o-transcribe")
        self.declare_parameter("env_file", "")
        self.declare_parameter("request_timeout_s", 30.0)
        self.declare_parameter("max_tool_rounds", 4)
        self.declare_parameter("max_history_items", 60)
        self.declare_parameter("max_consecutive_failures", 3)
        self.declare_parameter("continue_after_action", True)

        self.max_tool_rounds = int(self.get_parameter("max_tool_rounds").value)
        self.max_consecutive_failures = int(
            self.get_parameter("max_consecutive_failures").value
        )
        self.continue_after_action = bool(
            self.get_parameter("continue_after_action").value
        )
        self.conversation = Conversation(
            max_items=int(self.get_parameter("max_history_items").value)
        )

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
                self.decide(event)
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

        try:
            llm = self.get_llm()
        except Exception as exc:
            self.publish_reply("error", f"LLM을 초기화하지 못했습니다: {exc}")
            return

        for _ in range(self.max_tool_rounds):
            try:
                response = llm.respond(self.conversation.items())
            except Exception as exc:
                self.publish_reply("error", f"LLM 호출에 실패했습니다: {exc}")
                return

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

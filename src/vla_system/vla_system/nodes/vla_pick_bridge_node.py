#!/usr/bin/env python3
"""Hands agent decisions to cobot2_ws's pick_fsm and reflects its results back.

This node takes over ``vla_robot``'s ROS-facing role -- it subscribes
``/vla/robot/action``/``/vla/robot/stop``/``/vla/estop`` and publishes
``/vla/robot/state``, exactly like ``robot_node.py`` -- but instead of moving
an arm it forwards to a *different process in a different git clone*
(``~/cobot2_ws``'s ``vla_command_node``) over ``/vla/pick_command`` /
``/vla/pick_result`` (``std_msgs/String``, JSON). See
``docs/state.md`` "cobot2_ws 통합" for the full checklist this implements, and
``~/cobot2_ws/md/vla-bridge-contract.md`` for the schema this node is bound
by (not copied here on purpose -- CLAUDE.md #2).

🔴 Mutually exclusive with ``vla_robot`` (``enable_robot:=true``): both
subscribe the same action/stop topics and publish the same state topic. Two
processes racing to answer ``/vla/robot/action`` is not a supported
configuration -- launch one or the other, never both.

What this node deliberately does not do:

- **Base-frame coordinates.** ``object_id`` never crosses, and neither does
  ``vla_perception``'s table homography output -- cobot2_ws's own D435i +
  ``T_cam2base`` produces the actual grasp pose. A pixel *does* cross now
  (``pixel``/``pixel_wh``, contract #2/#8) to disambiguate which instance of
  a class, since the camera the pixel was measured on and cobot2_ws's own
  camera are confirmed the same physical D435i (2026-08-10/11) -- see
  ``bbox_center()`` in ``bridge/pick_bridge.py``. cobot2_ws still ignores it
  today (no ``select_by_point()`` yet, contract #8).
- **Approval.** ``/pick/approve`` is never called, directly or indirectly --
  cobot2_ws's ``vla_command_node`` hard-rejects ``cmd:"approve"`` in code, and
  this bridge does not try to route around that (contract #4).
- **``pick_and_hold``/``release``.** cobot2_ws's FSM always carries a pick
  through to place; there is no "hold it" or "put it down here" on that side.
  Both are rejected locally before anything is published.
"""

import json
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from vla_interfaces.msg import RobotAction, RobotState, SceneSnapshot

from vla_system.bridge.pick_bridge import (
    UNSUPPORTED_ACTIONS,
    bbox_center,
    build_abort_command,
    build_pick_command,
    find_scene_object,
    parse_pick_result,
    place_rejection_reason,
    result_update,
)


class PickBridgeNode(Node):
    def __init__(self):
        super().__init__("vla_pick_bridge")

        self.declare_parameter("action_topic", "/vla/robot/action")
        self.declare_parameter("stop_topic", "/vla/robot/stop")
        self.declare_parameter("estop_topic", "/vla/estop")
        self.declare_parameter("state_topic", "/vla/robot/state")
        self.declare_parameter("scene_topic", "/vla/scene")
        # cobot2_ws's vla_command_node, a different process (different clone
        # entirely) -- these two must match its command_topic/result_topic
        # defaults exactly (voice_processing/vla_command_node.py).
        self.declare_parameter("pick_command_topic", "/vla/pick_command")
        self.declare_parameter("pick_result_topic", "/vla/pick_result")
        self.declare_parameter("max_scene_age_s", 2.0)
        # cobot2_ws's own wait_timeout_sec default is 50s (vla_command_node);
        # stay above it so a slow-but-real answer is never mistaken for a
        # dead one.
        self.declare_parameter("result_timeout_s", 60.0)
        # RobotState forces an opinion on "is motion enabled" even though
        # that switch lives in cobot2_ws's pick_fsm.yaml now, not here. True
        # because the point of this bridge is that motion really does
        # happen -- just on the other clone's arm.
        self.declare_parameter("motion_enabled", True)
        # vla-bridge-contract.md #5: table/discard are placeholder joint poses
        # on real hardware as of 2026-08-10, not taught to a safe place yet.
        # Off by default -- flip this once cobot2_ws confirms they're teach-
        # complete, rather than changing code to re-enable them.
        self.declare_parameter("allow_unverified_place", False)

        self.max_scene_age_s = float(self.get_parameter("max_scene_age_s").value)
        self.result_timeout_s = float(self.get_parameter("result_timeout_s").value)
        self.motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self.allow_unverified_place = bool(
            self.get_parameter("allow_unverified_place").value
        )

        self.scene: SceneSnapshot | None = None
        self.scene_monotonic = 0.0
        self.scene_lock = threading.Lock()

        # Only one request in flight, mirroring vla_robot's own rule: a
        # second action arriving while one is already out is rejected, not
        # queued -- a queue is how a stale target reaches the arm three
        # decisions after the user changed their mind.
        self.pending_request_id = ""
        self.pending_action: RobotAction | None = None
        self.pending_sent_monotonic = 0.0

        self.status = "idle"
        self.last_action_id = ""
        self.last_action = ""
        self.last_result = ""
        self.details = ""

        # RobotAction.header.stamp is when the agent read the world, not when
        # it finished thinking -- an LLM round-trip is seconds long, so a
        # stop can land inside it. Same guard as vla_robot's action_callback.
        self.stop_time_ns = 0

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.state_publisher = self.create_publisher(
            RobotState, str(self.get_parameter("state_topic").value), latched_qos
        )
        # command_qos here matches cobot2_ws's own COMMAND_QOS for these two
        # topics exactly (vla_command_node.py) -- RELIABLE/VOLATILE on both
        # sides, or the two processes never see each other's messages at all
        # (no error, just silence -- the classic ROS 2 QoS-mismatch symptom).
        self.pick_command_publisher = self.create_publisher(
            String, str(self.get_parameter("pick_command_topic").value), command_qos
        )
        self.create_subscription(
            String,
            str(self.get_parameter("pick_result_topic").value),
            self.pick_result_callback,
            command_qos,
        )
        self.create_subscription(
            SceneSnapshot,
            str(self.get_parameter("scene_topic").value),
            self.scene_callback,
            stream_qos,
        )
        self.create_subscription(
            RobotAction,
            str(self.get_parameter("action_topic").value),
            self.action_callback,
            command_qos,
        )
        # Two topics, one handler -- same split as vla_robot's for the same
        # reason: /vla/estop is the GUI's hardcoded keyword path, /vla/robot/
        # stop is the agent's cancel_current_action.
        for parameter in ("stop_topic", "estop_topic"):
            self.create_subscription(
                String,
                str(self.get_parameter(parameter).value),
                self.stop_callback,
                command_qos,
            )

        self.create_timer(1.0, self.check_result_timeout)

        self.publish_state()
        self.get_logger().info(
            "pick bridge ready: forwarding to cobot2_ws pick_fsm via "
            f"{self.get_parameter('pick_command_topic').value}"
        )

    # ------------------------------------------------------------ callbacks

    def scene_callback(self, message: SceneSnapshot) -> None:
        with self.scene_lock:
            self.scene = message
            self.scene_monotonic = time.monotonic()

    def action_callback(self, message: RobotAction) -> None:
        name = message.name.strip()

        if name in UNSUPPORTED_ACTIONS:
            self.reject(message, UNSUPPORTED_ACTIONS[name])
            return
        if name != "pick_and_place":
            self.reject(message, f"알 수 없는 동작입니다: {name}")
            return

        decided_ns = Time.from_msg(message.header.stamp).nanoseconds
        if decided_ns and decided_ns <= self.stop_time_ns:
            self.reject(message, "정지 이전에 결정된 동작이라 실행하지 않습니다")
            return

        if self.pending_action is not None:
            self.reject(message, "이미 다른 동작을 수행하는 중입니다")
            return

        object_id = message.object_id.strip()
        with self.scene_lock:
            scene = self.scene
            age = time.monotonic() - self.scene_monotonic
        if scene is None:
            self.reject(message, "아직 카메라 장면을 받지 못했습니다")
            return
        if age > self.max_scene_age_s:
            self.reject(message, f"카메라 장면이 {age:.1f}초 전 것이라 사용할 수 없습니다")
            return
        scene_object = find_scene_object(scene.objects, object_id)
        if scene_object is None:
            self.reject(message, f"'{object_id}'는 지금 화면에 없습니다")
            return
        class_name = scene_object.class_name

        # Optional (contract #2/#8): omitted while vla_perception hasn't
        # reported a frame resolution yet (image_width/height still 0, e.g.
        # right after startup before the first camera frame). cobot2_ws
        # validates but ignores both today (no select_by_point() yet) -- sent
        # anyway so this side doesn't need a second change once that lands.
        # The pixel means something on cobot2_ws's side without reprojection
        # only because the camera is now confirmed the same physical D435i
        # (2026-08-10/11) -- see bbox_center()'s docstring.
        pixel = None
        pixel_wh = None
        if scene.image_width and scene.image_height:
            pixel = bbox_center(scene_object)
            pixel_wh = (scene.image_width, scene.image_height)

        # Empty means the caller didn't set one (defensive default for
        # anything besides agent_node, which always fills it in now that
        # tools.py's pick_and_place requires place) -- basket is cobot2_ws's
        # own default too (contract #5), so this just makes it explicit here.
        place = message.place.strip() or "basket"
        reason_for_place_rejection = place_rejection_reason(
            place, allow_unverified=self.allow_unverified_place
        )
        if reason_for_place_rejection is not None:
            self.reject(message, reason_for_place_rejection)
            return

        payload = build_pick_command(
            class_name=class_name,
            place=place,
            request_id=message.action_id,
            stamp_ns=decided_ns,
            pixel=pixel,
            pixel_wh=pixel_wh,
        )
        self.pending_request_id = message.action_id
        self.pending_action = message
        self.pending_sent_monotonic = time.monotonic()
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self.get_logger().info(
            f"-> cobot2_ws pick_and_place class={class_name} place={place} "
            f"pixel={pixel} id={message.action_id}"
        )

        self.status = "moving"
        self.last_action_id = message.action_id
        self.last_action = message.name
        self.last_result = ""
        self.details = message.reason
        self.publish_state()

    def stop_callback(self, message: String) -> None:
        reason = message.data.strip() or "정지"
        self.stop_time_ns = self.get_clock().now().nanoseconds
        self.get_logger().warning(f"STOP: {reason}")

        # Fired unconditionally, same reasoning as vla_robot's stop_callback:
        # guessing "nothing is in flight, skip it" is the one mistake a stop
        # path is not allowed to make.
        payload = build_abort_command(
            request_id=uuid.uuid4().hex[:12],
            reason=reason,
            stamp_ns=self.get_clock().now().nanoseconds,
        )
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

        # No RobotState to publish here: with nothing pending there is
        # nothing to cancel, and with something pending, only cobot2_ws's own
        # terminal /vla/pick_result (pick_result_callback) can say what
        # actually happened to it -- inventing "cancelled" ahead of that
        # would be a guess this path is not allowed to make either.

    def pick_result_callback(self, message: String) -> None:
        doc = parse_pick_result(message.data)
        if doc is None:
            self.get_logger().warning(
                f"could not parse /vla/pick_result: {message.data!r}"
            )
            return
        request_id = str(doc.get("request_id", ""))
        if request_id != self.pending_request_id or not request_id:
            # A control-cmd (abort/start/reset) ack, or a late reply for a
            # request already given up on (see check_result_timeout). Not
            # this node's concern.
            self.get_logger().debug(f"ignoring unmatched pick_result id={request_id!r}")
            return

        action = self.pending_action
        update = result_update(str(doc.get("result", "")))

        if not update.terminal:
            self.status = update.status
            self.publish_state()
            return

        self.pending_request_id = ""
        self.pending_action = None
        self.pending_sent_monotonic = 0.0
        self.status = update.status
        self.last_action_id = action.action_id if action else self.last_action_id
        self.last_action = action.name if action else self.last_action
        self.last_result = update.last_result
        self.details = str(doc.get("reason", ""))
        self.publish_state()

    def check_result_timeout(self) -> None:
        if self.pending_action is None:
            return
        age = time.monotonic() - self.pending_sent_monotonic
        if age <= self.result_timeout_s:
            return
        self.get_logger().error(
            f"cobot2_ws did not answer /vla/pick_command within "
            f"{self.result_timeout_s:.0f}s (id={self.pending_request_id}); giving up"
        )
        action = self.pending_action
        self.pending_request_id = ""
        self.pending_action = None
        self.pending_sent_monotonic = 0.0
        self.status = "idle"
        self.last_action_id = action.action_id
        self.last_action = action.name
        self.last_result = "failed"
        self.details = "cobot2_ws로부터 응답이 없습니다"
        self.publish_state()

    # -------------------------------------------------------------- outputs

    def reject(self, action: RobotAction, reason: str) -> None:
        self.get_logger().warning(f"action rejected ({action.name}): {reason}")
        self.last_action_id = action.action_id
        self.last_action = action.name
        self.last_result = "rejected"
        self.details = reason
        self.publish_state()

    def publish_state(self) -> None:
        message = RobotState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base"
        message.status = self.status
        # cobot2_ws's /vla/pick_result carries no "what is the gripper
        # holding" field (vla-bridge-contract.md #3), so this bridge cannot
        # populate holding_object_id/holding_class_name -- see docs/state.md
        # "cobot2_ws 통합" for the open question this leaves (subscribing
        # cobot2_ws's /pick/state for VERIFY/LIFT/PLACE would close it).
        message.holding_object_id = ""
        message.holding_class_name = ""
        message.current_action_id = (
            self.pending_action.action_id if self.pending_action else ""
        )
        message.current_action = (
            self.pending_action.name if self.pending_action else ""
        )
        message.last_action_id = self.last_action_id
        message.last_action = self.last_action
        message.last_result = self.last_result
        message.details = self.details
        message.motion_enabled = self.motion_enabled
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PickBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

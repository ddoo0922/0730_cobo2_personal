"""로봇도 카메라도 없이 GUI로 판단 계층을 만져 보는 무대.

    ros2 run vla_system agent_node --ros-args -p skill_tier_enabled:=true &
    python3 -m eval.dryrun_stage &
    ros2 run vla_system vla_gui

무엇을 흉내내는가
----------------
`perception_node` 대신 고정된 테이블을 계속 내보내고, `robot_node` 대신 동작을
받아 성공했다고 답한다. 판단 계층 입장에서는 카메라와 팔이 붙어 있는 것과
구분되지 않는다 -- 같은 토픽, 같은 메시지, 같은 순서다.

무엇을 흉내내지 않는가
--------------------
물리 시간, 파지 실패, 추적 번호 재배정, 30Hz로 흔들리는 장면. 실기에서 처음
드러날 것들이 정확히 이것들이라 여기서 잘 도는 것이 실기 보증이 되지는 않는다.
그림판에서 확인할 수 있는 것은 **말이 오가는 흐름**까지다.

집힌 물체는 테이블에서 사라진다. 그래야 "사과 다 담아줘"가 끝난다.
"""

from __future__ import annotations

import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vla_interfaces.msg import RobotAction, RobotState, SceneObject, SceneSnapshot

# (id, class, color, x, y) -- 위치는 GUI 표시용이라 대충이어도 된다.
TABLE = [
    ("apple_1", "apple", "red", 0.52, 0.18),
    ("apple_2", "apple", "red", 0.52, 0.02),
    ("banana_3", "banana", "yellow", 0.52, -0.12),
    ("cup_4", "cup", "white", 0.58, -0.24),
    ("scissors_5", "scissors", "black", 0.46, 0.30),
]


def latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Stage(Node):
    def __init__(self, action_seconds: float) -> None:
        super().__init__("dryrun_stage")
        self.action_seconds = action_seconds
        self.present = [row[0] for row in TABLE]
        self.holding = ""
        self.lock = threading.Lock()

        self.scene_publisher = self.create_publisher(SceneSnapshot, "/vla/scene", latched())
        self.state_publisher = self.create_publisher(RobotState, "/vla/robot/state", latched())
        self.create_subscription(RobotAction, "/vla/robot/action", self._on_action, 10)
        self.create_subscription(String, "/vla/robot/stop", self._on_stop, 10)
        self.create_subscription(String, "/vla/estop", self._on_stop, 10)

        self.create_timer(0.5, self._publish_scene)
        self._publish_state("idle")
        self.get_logger().info(
            f"무대 준비: {', '.join(self.present)} · 동작 {action_seconds:.1f}초")

    # ------------------------------------------------------------- 장면

    def _publish_scene(self) -> None:
        message = SceneSnapshot()
        message.header.stamp = self.get_clock().now().to_msg()
        message.camera_frame = "dryrun"
        message.calibration_ok = True
        with self.lock:
            present = list(self.present)
        for index, (object_id, class_name, color, x, y) in enumerate(TABLE, 1):
            if object_id not in present:
                continue
            item = SceneObject()
            item.id = object_id
            item.track_id = index
            item.class_name = class_name
            item.confidence = 0.9
            item.color = color
            item.color_confidence = 0.9
            item.x_min, item.y_min, item.x_max, item.y_max = 0.0, 0.0, 10.0, 10.0
            item.position_valid = True
            item.position_base.x, item.position_base.y, item.position_base.z = x, y, 0.10
            item.depth_m = 0.5
            message.objects.append(item)
        self.scene_publisher.publish(message)

    # ------------------------------------------------------------- 로봇

    def _publish_state(self, status: str, **kwargs) -> None:
        message = RobotState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = status
        message.motion_enabled = True
        message.holding_object_id = self.holding
        message.holding_class_name = next(
            (row[1] for row in TABLE if row[0] == self.holding), "")
        for key, value in kwargs.items():
            setattr(message, key, value)
        self.state_publisher.publish(message)

    def _on_action(self, message: RobotAction) -> None:
        self.get_logger().info(
            f"동작 수신: {message.name}({message.object_id}) place={message.place!r}")
        self._publish_state("moving", current_action=message.name,
                            current_action_id=message.action_id)
        threading.Timer(self.action_seconds, self._finish, args=[message]).start()

    def _finish(self, message: RobotAction) -> None:
        missed = ""
        with self.lock:
            if message.object_id and message.object_id in self.present:
                self.present.remove(message.object_id)
            elif message.object_id:
                # 이미 사라진 것을 집으려 했다. 실제 팔은 허공을 잡는다.
                missed = f"'{message.object_id}'는 이미 테이블에 없습니다."
        if message.name == "pick_and_hold" and not missed:
            self.holding = message.object_id
        elif message.name == "release":
            self.holding = ""
        self._publish_state(
            "holding" if self.holding else "idle",
            last_action=message.name, last_action_id=message.action_id,
            last_result="failed" if missed else "succeeded", details=missed)

    def _on_stop(self, message: String) -> None:
        self.get_logger().info(f"정지: {message.data}")
        self._publish_state("idle", last_action="stop", last_result="cancelled",
                            details="사용자 정지")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action-seconds", type=float, default=1.5,
                        help="동작 하나가 걸리는 시간. 끼어들기를 시험하려면 늘린다")
    args = parser.parse_args()

    rclpy.init()
    node = Stage(args.action_seconds)
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

"""로봇도 카메라도 없이 GUI로 판단 계층을 만져 보는 무대.

    ros2 run vla_system agent_node --ros-args -p skill_tier_enabled:=true &
    python3 -m eval.dryrun_stage &
    ros2 run vla_system vla_gui

무엇을 흉내내는가
----------------
`perception_node` 대신 고정된 테이블을 계속 내보내고, `robot_node` 대신 동작을
받아 성공했다고 답한다. 판단 계층 입장에서는 카메라와 팔이 붙어 있는 것과
구분되지 않는다 -- 같은 토픽, 같은 메시지, 같은 순서다.

라벨이 그려진 화면(`/vla/perception/annotated_image`)도 함께 낸다. 그 위에
**화살표 하나**를 그리는데, 사람이 손으로 가리키는 것의 대역이다. "이거 집어줘"는
화살표 없이는 시험할 수 없다 -- 그 발화의 뜻이 전부 거기에 있기 때문이다.

    ros2 topic pub --once /dryrun/point_at std_msgs/String "{data: 'cup_4'}"

로 가리키는 대상을 바꾼다. 빈 문자열이면 화살표가 사라지고, 그때 "이거 집어줘"는
되물어야 맞다(가리키는 것이 없으니 짐작하면 안 된다).

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

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
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

FRAME_SIZE = (480, 640)          # 실제 카메라와 같은 크기
BOX_HALF = 55                    # 박스 반폭(px)

# 색은 BGR. 진짜 인식 결과가 아니라 사람이 화면에서 알아보라고 넣은 것이다.
SWATCH = {"red": (60, 60, 220), "yellow": (60, 200, 230),
          "white": (200, 200, 200), "black": (60, 60, 60)}


def table_to_pixel(x: float, y: float) -> tuple[int, int]:
    """로봇 좌표(m) -> 화면 좌표(px). 보기 좋게만 맞춘 가짜 사영이다."""
    column = int(round(320 - y * 900))
    row = int(round(1150 - x * 1500))
    return (max(BOX_HALF, min(FRAME_SIZE[1] - BOX_HALF, column)),
            max(BOX_HALF, min(FRAME_SIZE[0] - BOX_HALF, row)))


def arrow_for(column: int, row: int, length: int = 130):
    """물체를 찌르는 화살표의 (꼬리, 촉). 사람 손가락 대역이다.

    방향을 고르는 이유: 아래에서만 찌르면 화면 아래쪽 물체는 꼬리가 프레임
    밖으로 나가 **사실상 안 그려진다.** 그림에 화살표가 없으면 모델은 짐작하지
    않고 되묻는데(그게 맞는 동작이다), 시험하는 쪽은 "가리켰는데 왜 되묻지?"로
    읽게 된다 -- 무대의 결함이 제품의 결함처럼 보인다. 2026-08-11에 가위가
    정확히 그랬다.
    """
    gap = BOX_HALF + 8
    height, width = FRAME_SIZE
    # 아래 -> 위 -> 오른쪽 -> 왼쪽 순으로, 꼬리가 프레임 안에 들어오는 첫 방향.
    for tail, tip in (
        ((column, row + gap + length), (column, row + gap)),
        ((column, row - gap - length), (column, row - gap)),
        ((column + gap + length, row), (column + gap, row)),
        ((column - gap - length, row), (column - gap, row)),
    ):
        if 0 <= tail[0] < width and 0 <= tail[1] < height:
            return tail, tip
    return (column, row + gap + length), (column, row + gap)


def latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stream(depth: int = 1) -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


class Stage(Node):
    def __init__(self, action_seconds: float, point_at: str,
                 position_valid: bool = True) -> None:
        super().__init__("dryrun_stage")
        self.action_seconds = action_seconds
        # cobot2_ws 연동에서는 이 ws가 테이블 보정을 하지 않아 3D 위치가 안
        # 잡힌다 -- pick_fsm이 클래스 이름만 받아 자기 카메라로 좌표를 낸다.
        # 그 구성을 흉내내려면 False. 판단 계층이 position_valid에 기대고 있으면
        # 여기서 드러난다(2026-08-11 병합에서 실제로 그랬다).
        self.position_valid = position_valid
        self.present = [row[0] for row in TABLE]
        self.holding = ""
        self.point_at = point_at
        self.lock = threading.Lock()

        self.scene_publisher = self.create_publisher(SceneSnapshot, "/vla/scene", latched())
        self.state_publisher = self.create_publisher(RobotState, "/vla/robot/state", latched())
        # 판단 계층이 이 토픽을 BEST_EFFORT로 잡는다(진짜 perception과 같은 QoS).
        # 여기서 RELIABLE로 내면 아무 오류 없이 영영 안 만난다.
        self.image_publisher = self.create_publisher(
            Image, "/vla/perception/annotated_image", stream())
        self.create_subscription(RobotAction, "/vla/robot/action", self._on_action, 10)
        self.create_subscription(String, "/vla/robot/stop", self._on_stop, 10)
        self.create_subscription(String, "/vla/estop", self._on_stop, 10)
        self.create_subscription(String, "/dryrun/point_at", self._on_point_at, 10)

        self.create_timer(0.5, self._publish_scene)
        self._publish_state("idle")
        self.get_logger().info(
            f"무대 준비: {', '.join(self.present)} · 동작 {action_seconds:.1f}초 · "
            f"가리키는 것: {self.point_at or '(없음)'} · "
            f"3D 위치 {'있음' if self.position_valid else '없음(cobot2_ws 연동 구성)'}")

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
            item.position_valid = self.position_valid
            item.position_base.x, item.position_base.y, item.position_base.z = x, y, 0.10
            item.depth_m = 0.5
            message.objects.append(item)
        self.scene_publisher.publish(message)
        self._publish_annotated(present, message.header)

    def _publish_annotated(self, present: list[str], header) -> None:
        """라벨 그려진 화면. 박스 위 글자가 곧 scene의 id다 -- 그 둘이 같은
        문자열이라는 것이 모델이 그림과 JSON을 잇는 유일한 끈이다."""
        frame = np.full((*FRAME_SIZE, 3), 240, dtype=np.uint8)
        centres = {}
        for object_id, class_name, color, x, y in TABLE:
            if object_id not in present:
                continue
            column, row = table_to_pixel(x, y)
            centres[object_id] = (column, row)
            swatch = SWATCH.get(color, (120, 120, 120))
            cv2.rectangle(frame, (column - BOX_HALF, row - BOX_HALF),
                          (column + BOX_HALF, row + BOX_HALF), swatch, -1)
            cv2.rectangle(frame, (column - BOX_HALF, row - BOX_HALF),
                          (column + BOX_HALF, row + BOX_HALF), (40, 40, 40), 2)
            cv2.putText(frame, object_id, (column - BOX_HALF, row - BOX_HALF - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 2, cv2.LINE_AA)

        with self.lock:
            target = self.point_at
        if target in centres:
            tail, tip = arrow_for(*centres[target])
            cv2.arrowedLine(frame, tail, tip, (20, 20, 20), 6, tipLength=0.35)

        message = Image()
        message.header = header
        message.height, message.width = frame.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = frame.shape[1] * 3
        message.data = frame.tobytes()
        self.image_publisher.publish(message)

    def _on_point_at(self, message: String) -> None:
        target = message.data.strip()
        with self.lock:
            self.point_at = target
        self.get_logger().info(f"가리키는 것: {target or '(없음)'}")

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
    parser.add_argument("--point-at", default="apple_2",
                        help="화살표가 가리킬 물체 id. 빈 문자열이면 안 그린다")
    parser.add_argument("--no-position", action="store_true",
                        help="position_valid를 false로. cobot2_ws 연동 구성을 흉내낸다 "
                             "-- 이 ws가 테이블 보정을 안 하는 상태다")
    args = parser.parse_args()

    rclpy.init()
    node = Stage(args.action_seconds, args.point_at,
                 position_valid=not args.no_position)
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

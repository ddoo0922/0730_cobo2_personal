#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goal_setter_replan.py — NVIDIA 공식 예제(isaac_ros_moveit_goal_setter) 방식의 재계획 루프

reactive_replan.py 와 **같은 목표를 다른 방식으로** 수행한다. 둘을 같은 목표로 돌려
비교하는 것이 이 파일의 존재 이유다.

    reactive_replan.py : plan_only=True  + JTC 액션 직접 (MoveIt 실행관리자 우회)
    이 파일            : plan_only=False + move_group 이 실행까지 (예제와 동일)

베낀 원본 (도커 마운트 실제 소스)
---------------------------------
isaac_ros-dev/src/isaac_ros_cumotion/isaac_ros_moveit_goal_setter/
    ├── move_group_client.py    ← MoveGroupClient 구조·필드 세팅을 그대로 따랐다
    └── goal_initializer.py     ← 타이머 기반 재계획 루프 구조를 따랐다

원본과 **의도적으로 다르게** 한 곳 (전부 이유가 있다)
------------------------------------------------------
① 🔴 `_get_joint_constraints()` 의 `return` 누락 — **원본의 버그다.**
   move_group_client.py:60-66 이 Constraints 를 만들어 append 만 하고 return 이 없어
   None 을 돌려준다. 그대로 쓰면 `goal_constraints.append(None)` 이 되어 관절 목표가 깨진다.
   여기서는 고쳐서 썼다.

② `goal_change_position_threshold` 기본값 0.0 (원본은 0.1 m)
   원본 goal_initializer.py:119 는 **목표가 10 cm 넘게 움직였을 때만** 재계획한다.
   원본은 "움직이는 목표 추종(object following)"이 목적이라 그게 맞다.
   우리는 **목표 고정 + 장애물이 움직임**이라 그 조건을 켜면 재계획이 한 번도 안 일어난다.
   그래서 기본값을 0.0(항상 재계획)으로 둔다. 원본 거동을 보려면 0.1 로 올린다.

③ 관절 목표를 지원한다 (원본은 TF 의 grasp_frame → pose 목표 전용)
   ⚠️ cuMotion 은 관절 목표도 내부에서 FK 로 EE pose 를 만든 뒤 **IK 를 푼다**
      (cumotion_planner.py:714-730). 그래서 관절 목표인데도 IK_FAIL 이 날 수 있다.

④ `mode` 파라미터로 순차/선점 두 거동을 고를 수 있게 했다 (원본은 순차뿐)
   원본 move_group_client.py:83-84 의 `while self._result is None: sleep(0.01)` 이
   **plan+execute 완료까지 블록**한다. 즉 원본은 실행이 끝나야 다음 계획을 던진다
   → **실행 중에는 장애물이 들어와도 궤적이 안 바뀐다.**
   그게 우리가 풀려는 문제 그 자체이므로, 비교를 위해 두 모드를 다 넣었다.

     mode:=sequential (기본, 원본과 동일)
         계획→실행 완료→다음 계획. **동적 회피가 안 되는 것을 보여주는 대조군.**
     mode:=preemptive
         완료를 안 기다리고 plan_timer_period 마다 새 goal 을 던진다.
         move_group 이 기존 goal 을 선점 교체해 주기를 기대한다.
         🔴 **이 거동은 검증 안 됐다.** move_group 이 선점을 거부하거나(goal reject),
            현재 궤적을 멈췄다 재시작할 수 있다. 그걸 보려고 만든 모드다.

관련 설정 (실측 확인)
---------------------
· m0609_rg2_moveit/config/moveit_controllers.yaml : allowed_start_tolerance = **0.08** rad
  MoveIt 실행관리자가 "궤적 첫 점 vs 현재 로봇 상태" 오차를 이 값으로 검사한다.
  선점 교체가 이 검사에 걸리면 실행이 거부된다. 0.08(≈4.6°)은 두산 원본(0.01)보다 관대하다.
  ⚠️ reactive_replan.py 는 JTC 를 직접 불러 이 검사를 아예 안 탄다. 이게 두 파일의 핵심 차이다.
· cumotion_planner_manager.hpp:53 : canServiceRequest 가 planner_id == "cuMotion" 을 요구
  → planner_id 기본값을 'cuMotion' 으로 둔다(원본과 동일).

⚠️ 로봇이 실제로 움직인다. 첫 실행은 vel 을 낮추고 비상정지 버튼에 손을 올린 채로.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

D2R = math.pi / 180.0
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║                                                                           ║
# ║   🎯 목표 설정 — 여기만 고치면 된다. ①과 ② 중 **하나만** 주석을 푼다.      ║
# ║                                                                           ║
# ║   (ros2 run ... --ros-args -p goal_type:=pose 처럼 실행 인자로 줘도        ║
# ║    되고, 그때는 인자가 아래 값을 이긴다)                                   ║
# ║                                                                           ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

# ┌───────────────────────────────────────────────────────────────────────────┐
# │ ① 관절값으로 목표를 준다 (deg)                            │
# └───────────────────────────────────────────────────────────────────────────┘
# GOAL_MODE = 'joint'

GOAL_A_JOINT_DEG = [45.0, 0.0, 90.0, 0.0, 90.0, 0.0]
GOAL_B_JOINT_DEG = [-45.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# ⚠️ "관절 목표"라고 부르지만 cuMotion 은 이걸 FK 로 pose 로 바꾼 뒤 **IK 를 다시 푼다**
#    (cumotion_planner.py:714-730). 같은 pose 를 만드는 다른 IK 해로 갈 수 있으므로
#    **여기 적은 관절값 그대로 도착한다는 보장이 없다.** 관절 목표인데 IK_FAIL 도 난다.

# ┌───────────────────────────────────────────────────────────────────────────┐
# │ ② base_link 기준 xyz 로 목표를 준다                           │
# │    GOAL_MODE 줄의 주석을 풀고, 위 ①의 GOAL_MODE 줄을 주석 처리한다        │
# └───────────────────────────────────────────────────────────────────────────┘
GOAL_MODE = 'pose'

# 형식 두 가지를 다 받는다 — **길이로 자동 판별**한다:
#     7개 → [x, y, z, qx, qy, qz, qw]   쿼터니언 (권장)
#     6개 → [x, y, z, roll, pitch, yaw] 오일러, 각도는 **deg**
#
# 🔴 A/B 자세는 pitch 가 정확히 90° 라 **짐벌락 지점**이다. rpy 로 적으면 같은 자세를
#    나타내는 (roll, yaw) 조합이 무수히 많아 헷갈린다 → **쿼터니언(7개)을 권한다.**
#
# 아래 값은 URDF 로 FK 계산한 것이다(위 관절값 A/B 와 같은 자세). **실측이 아니다.**
# 실기에서 확인하려면 로봇을 그 자세에 두고:
#     ros2 run tf2_ros tf2_echo base_link tool0
# 이 출력의 Translation + Rotation(Quaternion)을 그대로 복사한다. 그게 단일 출처다.
#
# ⚠️ 기준 링크는 **tool0** 이지 RG2 손가락 끝(TCP)이 아니다 (m0609_rg2.xrdf 주석).
# ⚠️ 두산 API 의 posx(x,y,z,a,b,c)를 그대로 옮기면 안 된다. 두산은 **mm + Euler ZYZ**
#    (dsr_common2/include/DRFS.h:769), 여기는 **m + 쿼터니언**이다. 숫자가 6개로
#    똑같이 생겨서 조용히 틀린다.
GOAL_A_POSE = [0.2558, 0.2646, 0.4244, -0.653298, -0.270371, 0.653320, -0.270692]
GOAL_B_POSE = [0.2646, -0.2558, 0.4244, -0.653132, 0.270770, 0.653376, 0.270559]

# ═══════════════════════ 목표 설정 끝 ═══════════════════════════════════════


def err_name(code: int) -> str:
    for name in dir(MoveItErrorCodes):
        if name.startswith('_') or name == 'val':
            continue
        try:
            if getattr(MoveItErrorCodes, name) == code:
                return f'{name}({code})'
        except TypeError:
            continue
    return f'UNKNOWN({code})'


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class MoveGroupExecClient:
    """예제의 MoveGroupClient 와 같은 역할.

    🔴 원본과의 결정적 공통점: `plan_only = False`.
       계획만 받아오는 게 아니라 **move_group 이 컨트롤러로 실행까지** 한다
       (MoveItSimpleControllerManager → /dsr01/dsr_moveit_controller/follow_joint_trajectory).
       그래서 이 파일은 컨트롤러 액션 이름을 몰라도 된다 — MoveIt 이 알아서 찾는다.
    """

    def __init__(self, node: Node, group_name: str, pipeline_id: str,
                 planner_id: str, ee_link: str, base_frame: str):
        self._node = node
        self._group_name = group_name
        self._pipeline_id = pipeline_id
        self._planner_id = planner_id
        self._ee_link = ee_link
        self._base_frame = base_frame

        self._cb = ReentrantCallbackGroup()
        self._client = ActionClient(node, MoveGroup, '/move_action', callback_group=self._cb)

        self._result = None
        self._goal_handle = None
        self._lock = threading.Lock()

    def wait_for_server(self, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while not self._client.wait_for_server(timeout_sec=1.0):
            if time.time() - t0 > timeout:
                self._node.get_logger().error(
                    '/move_action 액션 서버 없음 → move_group(T7) 미기동')
                return False
            self._node.get_logger().info('move_action 대기 중...')
        return True

    # ── 목표 구성 ───────────────────────────────────────────────────────────
    def joint_constraints(self, positions: Sequence[float],
                          joint_names: Sequence[str]) -> Constraints:
        """🔴 원본(move_group_client.py:60-66)은 return 이 빠져 None 을 돌려준다 — 버그.
        여기서는 constraints 를 제대로 돌려준다."""
        constraints = Constraints()
        for position, joint_name in zip(positions, joint_names):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            # 원본은 tolerance/weight 를 안 채운다. cuMotion 은 position 만 읽으므로
            # (cumotion_planner.py:714-722) 동작은 하지만, ompl 로 바꿔 쓸 때를 위해 채운다.
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def pose_constraints(self, pose: PoseStamped) -> Constraints:
        """원본 _get_pose_constraints 와 같은 구성.

        ⚠️ 원본은 constraint_region.primitives 를 안 채우고 primitive_poses 만 채운다.
           cuMotion 은 primitive_poses[0].position 만 읽으므로(cumotion_planner.py:737-743)
           그래도 동작하지만, pipeline_id:=ompl 로 바꾸면 영역이 비어 실패한다.
           그래서 여기서는 SPHERE 를 하나 넣어 둔다.
        """
        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = pose.header.frame_id
        pc.link_name = self._ee_link
        pc.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])]
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = pose.header.frame_id
        oc.link_name = self._ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        return constraints

    # ── 목표 전송 ───────────────────────────────────────────────────────────
    def send_goal(self, constraints: Constraints,
                  vel_scale: float, acc_scale: float,
                  allowed_planning_time: float = 10.0,
                  wait: bool = True, wait_timeout: float = 120.0,
                  replan: bool = False, replan_attempts: int = 3,
                  replan_delay: float = 0.5):
        """MoveGroup goal 을 보낸다.

        wait=True  : 원본과 동일. plan+execute 가 **끝날 때까지 블록**하고 결과를 돌려준다.
        wait=False : 보내고 바로 리턴(선점 실험용). 결과는 last_result() 로 나중에 본다.
        """
        with self._lock:
            self._result = None

        goal_msg = MoveGroup.Goal()
        goal_msg.request.planner_id = self._planner_id      # 'cuMotion' (canServiceRequest)
        goal_msg.request.pipeline_id = self._pipeline_id    # 'isaac_ros_cumotion'
        goal_msg.request.group_name = self._group_name
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = allowed_planning_time
        # 🔴 원본은 scaling 을 안 건드린다(=0.0 → cuMotion 이 자기 time_dilation_factor 사용).
        #    우리는 실기 속도를 통제해야 하므로 명시한다. cumotion_planner.yaml 이
        #    override_moveit_scaling_factors:false 라 여기서 보낸 값이 이긴다.
        goal_msg.request.max_velocity_scaling_factor = vel_scale
        goal_msg.request.max_acceleration_scaling_factor = acc_scale

        # 🔴 이 한 줄이 reactive_replan.py 와의 근본 차이다.
        #    False = move_group 이 계획하고 **실행까지** 한다(실행관리자·start tolerance 경유).
        goal_msg.planning_options.plan_only = False

        # ═══ 🔴 STOP_REPLAN — MoveIt 내장 동적 회피 ═══════════════════════════
        #   PlanningOptions.msg 원문:
        #     "If the plan becomes invalidated during execution, it is possible to have
        #      that plan recomputed and execution restarted. This flag enables this functionality"
        #
        #   즉 move_group 의 plan_execution 이 **실행 중 planning scene 을 감시**하다가
        #   남은 경로가 무효가 되면 멈추고 다시 계획한다. 우리가 손으로 만든 루프와
        #   같은 일을 MoveIt 이 대신 해준다.
        #
        #   ✅ 2026-08-08 실기 확인: pick_fsm(octomap + OMPL + replan:true) 에서
        #      이동 중 장애물을 넣으면 **멈췄다가 재계산해 우회**했다. 이 기능은 동작한다.
        #
        #   🔴 감시와 계획이 **다른 지도**를 보면 라이브락이 날 수 있다:
        #        감시  = MoveIt planning scene = **octomap** (해상도 0.1, padding 부풀리기)
        #        계획  = cuMotion              = **nvblox ESDF** (해상도 0.05, T4 세그멘터)
        #      octomap 이 "이 경로 무효"라고 거부했는데 cuMotion 은 nvblox 기준으로
        #      문제없다며 **같은 경로를 다시 낸다** → 무한 반복 → replan_attempts 소진.
        #      cuMotion 은 왜 거부당했는지 모르고, 다른 지도를 보니 고칠 수도 없다.
        #
        #      ⚠️ 이미 그 불일치의 증거가 있다: MoveIt 은 계획 직후에도 cuMotion 궤적을
        #         planning scene 으로 재검증하고(check_solution_paths_), 그게
        #         INVALID_MOTION_PLAN(-2) 다 — README 0-2 절 실패율 6~17%.
        #         replan 은 그 결합을 **실행 구간까지 확장**한다.
        #
        #   실험 순서: pipeline:=ompl 로 먼저(감시·계획이 같은 지도 → 라이브락 없음)
        #              → pipeline:=isaac_ros_cumotion 으로 하이브리드 시험
        goal_msg.planning_options.replan = bool(replan)
        goal_msg.planning_options.replan_attempts = int(replan_attempts)
        goal_msg.planning_options.replan_delay = float(replan_delay)

        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

        if not wait:
            return None

        deadline = time.time() + wait_timeout
        while rclpy.ok():
            with self._lock:
                if self._result is not None:
                    return self._result
            if time.time() > deadline:
                self._node.get_logger().error(f'결과 대기 {wait_timeout:.0f}s 초과')
                return None
            time.sleep(0.01)
        return None

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # 🔴 선점 모드에서 여기가 찍히면 "move_group 은 동시 goal 을 거부한다"는 뜻이다.
            #    그 경우 예제 방식으로는 실행 중 궤적 교체가 원리적으로 불가능하다.
            self._node.get_logger().error(
                'move_group 이 goal 을 거부했다 (선점 모드면 = 동시 goal 불가)')
            with self._lock:
                self._result = 'REJECTED'
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_cb)

    def _get_result_cb(self, future):
        result = future.result().result
        with self._lock:
            self._result = result

    def last_result(self):
        with self._lock:
            return self._result

    def cancel(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None


class GoalSetterReplanNode(Node):
    """예제 goal_initializer.py 의 타이머 루프 구조를 따른 노드."""

    def __init__(self):
        super().__init__('goal_setter_replan')

        p = self.declare_parameter
        # sequential = 원본 거동(실행 완료까지 대기) / preemptive = 완료 안 기다리고 새 goal
        self.mode = p('mode', 'sequential').value
        # 원본 goal_initializer.py:50 과 같은 이름·기본값
        self.plan_timer_period = float(p('plan_timer_period', 2.0).value)
        # 원본은 0.1 m. 목표 고정인 우리 용도에서는 0.0(항상 재계획)이 맞다 — 파일 상단 ② 참고
        self.goal_change_position_threshold = float(
            p('goal_change_position_threshold', 0.0).value)

        self.group_name = p('planner_group_name', 'manipulator').value
        self.pipeline_id = p('pipeline_id', 'isaac_ros_cumotion').value
        self.planner_id = p('planner_id', 'cuMotion').value        # canServiceRequest 요구값
        self.ee_link = p('end_effector_link', 'tool0').value
        self.base_frame = p('world_frame', 'base_link').value

        self.vel_scale = float(p('vel_scale', 0.15).value)
        self.acc_scale = float(p('acc_scale', 0.15).value)
        self.allowed_planning_time = float(p('allowed_planning_time', 10.0).value)

        # 🎯 기본값은 전부 파일 상단의 "목표 설정" 블록에서 온다. 거기를 고치면 된다.
        #    실행 인자(-p goal_type:=pose 등)를 주면 그쪽이 이긴다.
        self.goal_type = p('goal_type', GOAL_MODE).value             # joint | pose
        self.goal_joint_deg = list(p('goal_joint_deg', GOAL_A_JOINT_DEG).value)
        self.goal_pose = list(p('goal_pose', GOAL_A_POSE).value)

        self.pingpong = bool(p('pingpong', False).value)
        self.pingpong_b_deg = list(p('pingpong_b_deg', GOAL_B_JOINT_DEG).value)
        self.goal_pose_b = list(p('goal_pose_b', GOAL_B_POSE).value)
        self.cycles = int(p('cycles', 0).value)                     # 0 = 무한

        # 🎯 A→B 를 N 등분해서 경유점을 찍는다. 1 = 직행(지금까지의 거동).
        #
        #   왜: plan() 호출 하나가 곧 ESDF 재조회다. 구간을 쪼개면 장애물 지도를 그만큼
        #   자주 읽으므로 **회피 반응 시간이 1/N 로 줄어든다.** 구간 안에서는 여전히 못 피한다.
        #
        #   2026-08-08 실측 기반 계산 (vel/acc 0.25, pose 모드, A↔B 90°):
        #     구간 실행시간 = max(전체이동 7.6초 / N, 바닥값) + MoveIt 오버헤드 0.55초
        #     바닥값 = num_trajopt_time_steps(32) × interpolation_dt(0.025) / vel_scale
        #            = 0.8 / 0.25 = 3.2초   ← 아무리 잘게 쪼개도 이 밑으로 안 내려간다
        #
        #     N=1  총 8.2초 / 반응 8.2초      N=2  총 8.7초 / 반응 4.4초  ← 사실상 공짜
        #     N=3  총 11.3초 / 반응 3.8초     N=4+ 총 15초+ / 반응 3.8초  ← 손해만 커진다
        #
        #   ⚠️ 3.8초보다 빠르게 하려면 vel_scale 을 더 올리거나(회피 여유는 줄어든다)
        #      T6 의 num_trajopt_time_steps 를 줄여야 한다(미검증).
        #   ⚠️ 경유점은 **고정점**이다. 장애물이 경유점 위에 앉으면 그 구간 계획이 실패한다.
        #      목표가 먼 진짜 재계획과 달리 우회할 자유가 없다. N 을 키울수록 이 위험이 커진다.
        #   ⚠️ pingpong 일 때만 동작한다 (A/B 양 끝점을 알아야 사이를 나눌 수 있다).
        self.waypoints = max(1, int(p('waypoints', 1).value))

        self.joint_goal_tol = float(p('joint_goal_tol', 0.01).value)
        # 🔴 목표 하나(pingpong=false)일 때 도착하면 종료한다.
        #    이게 없으면 도착한 뒤에도 타이머가 같은 goal 을 영원히 다시 던진다
        #    (2026-08-07 실측: 이미 목표에 있는데도 사이클마다 SUCCESS 가 5.8초씩 찍혔다).
        self.stop_when_arrived = bool(p('stop_when_arrived', True).value)
        # 🔴 preemptive 전용 실험 스위치. 새 goal 전에 이전 goal 을 취소할지.
        #    기본 false = 2026-08-08 에 실측한 것과 같은 조건(비교 기준을 유지한다).
        self.cancel_previous = bool(p('cancel_previous', False).value)

        # ═══ 🔴 STOP_REPLAN — MoveIt 내장 동적 회피 (send_goal 주석이 본체다) ═══
        #   true 면 move_group 이 실행 중 planning scene 을 감시하다가 경로가 무효가 되면
        #   멈추고 다시 계획한다. **우리가 손으로 만든 루프와 같은 일을 MoveIt 이 한다.**
        #   ✅ pick_fsm(octomap+OMPL)에서 실기 확인됨 — 멈췄다가 재계산해 우회
        #   🔴 pipeline:=isaac_ros_cumotion 과 함께 켜면 감시(octomap)와 계획(nvblox)이
        #      다른 지도를 봐서 라이브락 위험이 있다. 그걸 보려는 게 이 실험이다.
        #   기본 false = 지금까지 실측한 조건(비교 기준 유지)
        self.replan = bool(p('replan', False).value)
        self.replan_attempts = int(p('replan_attempts', 3).value)
        self.replan_delay = float(p('replan_delay', 0.5).value)

        self._cb = ReentrantCallbackGroup()
        self._js_lock = threading.Lock()
        self._js_pos: dict = {}
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
            callback_group=self._cb)

        self.client = MoveGroupExecClient(
            self, self.group_name, self.pipeline_id, self.planner_id,
            self.ee_link, self.base_frame)

        self._previous_goal_position = None
        self._busy = False
        self._target_idx = 0
        self._wp_idx = 0            # 이번 구간에서 몇 번째 경유점을 향하고 있나
        self._first_leg = True      # 첫 구간은 출발점을 모르므로 경유점을 안 쓴다
        self._cycle_count = 0
        self.stat_goals = 0
        self.stat_fail = 0

        if self.mode not in ('sequential', 'preemptive'):
            self.get_logger().error(
                f"mode:={self.mode} 는 없다. sequential | preemptive 중 하나.")

        self.get_logger().info(
            f'모드={self.mode} / plan_timer_period={self.plan_timer_period}s / '
            f'vel={self.vel_scale} / goal_type={self.goal_type} / '
            f'waypoints={self.waypoints} / replan={self.replan}')

        if self.replan:
            # 🔴 replan 을 켜면 sequential 의 "실행 중엔 안 바뀐다"가 더 이상 참이 아니다.
            self.get_logger().warn(
                f'🔴 STOP_REPLAN 켜짐 (attempts={self.replan_attempts}, '
                f'delay={self.replan_delay}s) — move_group 이 실행 중 planning scene 을 '
                '감시하다가 경로가 무효가 되면 **멈추고 다시 계획한다.**')
            self.get_logger().warn(
                f'   감시 = MoveIt planning scene(octomap) / 계획 = {self.pipeline_id}')
            if self.pipeline_id != 'ompl':
                self.get_logger().warn(
                    '   ⚠️ 감시와 계획이 **다른 지도**를 본다(octomap vs nvblox). '
                    'octomap 이 거부한 경로를 cuMotion 이 같은 걸로 다시 낼 수 있다 '
                    '→ 멈춤-재계획 반복(라이브락). 그게 보이면 pipeline:=ompl 과 비교하라.')

        if self.mode == 'sequential' and not self.replan:
            self.get_logger().warn(
                '🔴 sequential + replan:=false — 실행이 끝나야 다음 계획을 던진다. '
                '즉 **실행 중에 장애물이 들어와도 궤적이 안 바뀐다.** 대조군용이다.')
        elif self.mode == 'preemptive':
            self.get_logger().warn(
                '⚠️ preemptive 는 검증 안 된 실험 모드다. move_group 이 동시 goal 을 '
                '거부하거나 궤적을 멈췄다 재시작할 수 있다. 그걸 보려는 게 목적이다.')

        # 원본 goal_initializer.py:71 과 같은 구조
        self.timer = self.create_timer(self.plan_timer_period, self.on_timer,
                                       callback_group=self._cb)

    def _on_joint_states(self, msg: JointState) -> None:
        with self._js_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._js_pos[name] = msg.position[i]

    def current_joints(self) -> List[float]:
        with self._js_lock:
            return [self._js_pos.get(j, 0.0) for j in JOINT_NAMES]

    def _current_target_deg(self) -> List[float]:
        if self.pingpong and self._target_idx == 1:
            return self.pingpong_b_deg
        return self.goal_joint_deg

    def _current_target_pose(self) -> List[float]:
        if self.pingpong and self._target_idx == 1:
            return self.goal_pose_b
        return self.goal_pose

    # ── 🎯 경유점 ────────────────────────────────────────────────────────────
    @staticmethod
    def _slerp(q0: Sequence[float], q1: Sequence[float], t: float) -> List[float]:
        """쿼터니언 구면 선형보간. 위치와 달리 회전은 단순 선형보간하면 안 된다."""
        q1 = list(q1)
        d = sum(a * b for a, b in zip(q0, q1))
        if d < 0.0:                       # 더 짧은 쪽으로 돈다
            q1 = [-v for v in q1]
            d = -d
        if d > 0.9995:                    # 거의 같으면 선형으로 충분(수치 안정)
            q = [a + t * (b - a) for a, b in zip(q0, q1)]
        else:
            th0 = math.acos(max(-1.0, min(1.0, d)))
            s0 = math.sin(th0)
            q = [math.sin(th0 - th0 * t) / s0 * a + math.sin(th0 * t) / s0 * b
                 for a, b in zip(q0, q1)]
        n = math.sqrt(sum(v * v for v in q))
        return [v / n for v in q]

    def _pose_as_quat(self, g: Sequence[float]) -> Optional[tuple]:
        """goal_pose(6개 rpy 또는 7개 쿼터니언)를 (xyz, quat) 로 정규화한다."""
        if len(g) == 7:
            return [float(v) for v in g[:3]], [float(v) for v in g[3:]]
        if len(g) == 6:
            qm = quat_from_rpy(*[float(v) * D2R for v in g[3:]])
            return [float(v) for v in g[:3]], [qm.x, qm.y, qm.z, qm.w]
        return None

    def _sub_goal_ratio(self) -> float:
        """이번에 갈 경유점이 출발점에서 목표까지의 몇 % 지점인가."""
        return float(self._wp_idx + 1) / float(self.waypoints)

    def _leg_start(self):
        """이번 구간의 출발점 = 지금 목표의 반대쪽 끝점."""
        if self.goal_type == 'joint':
            return self.pingpong_b_deg if self._target_idx == 0 else self.goal_joint_deg
        return self.goal_pose_b if self._target_idx == 0 else self.goal_pose

    def _use_waypoints(self) -> bool:
        """경유점을 쓸 조건.

        🔴 첫 구간은 제외한다. 로봇이 실제로 어디서 출발하는지 모르는데 반대쪽 끝점에서
           보간하면 엉뚱한 지점이 나온다. 첫 목표에 도달한 뒤부터가 A↔B 사이라 의미가 있다.
        """
        return self.waypoints > 1 and self.pingpong and not self._first_leg

    def _build_constraints(self) -> Optional[Constraints]:
        if self.goal_type == 'joint':
            target_deg = self._current_target_deg()
            if self._use_waypoints():
                # 🎯 관절값 선형보간 — 출발점(반대쪽 끝)에서 목표까지 t 지점
                t = self._sub_goal_ratio()
                s = self._leg_start()
                target_deg = [a + t * (b - a) for a, b in zip(s, target_deg)]
            target = [v * D2R for v in target_deg]
            return self.client.joint_constraints(target, JOINT_NAMES)

        if self.goal_type == 'pose':
            g = self._current_target_pose()

            if self._use_waypoints():
                # 🎯 위치는 선형보간, 회전은 slerp. 둘 다 쿼터니언 형태로 맞춰서 섞는다.
                end = self._pose_as_quat(g)
                start = self._pose_as_quat(self._leg_start())
                if end is None or start is None:
                    self.get_logger().error(
                        'goal_pose 길이가 6 또는 7 이 아니다 — 경유점 보간 불가')
                    return None
                t = self._sub_goal_ratio()
                xyz = [a + t * (b - a) for a, b in zip(start[0], end[0])]
                quat = self._slerp(start[1], end[1], t)
                g = xyz + quat          # 항상 7개짜리가 된다

            ps = PoseStamped()
            ps.header.frame_id = self.base_frame       # world_frame, 기본 base_link
            ps.header.stamp = self.get_clock().now().to_msg()
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = [float(v) for v in g[:3]]

            # 🎯 길이로 자동 판별한다 (파일 상단 "목표 설정" 블록 참고)
            #    7개 → 쿼터니언 그대로 / 6개 → rpy(deg) 를 쿼터니언으로 변환
            if len(g) == 7:
                pose.orientation = Quaternion(
                    x=float(g[3]), y=float(g[4]), z=float(g[5]), w=float(g[6]))
            elif len(g) == 6:
                pose.orientation = quat_from_rpy(*[float(v) * D2R for v in g[3:]])
            else:
                self.get_logger().error(
                    f'goal_pose 길이가 {len(g)} 다. 7개(xyz+쿼터니언) 또는 6개(xyz+rpy deg)여야 한다.')
                return None

            ps.pose = pose

            # 원본 goal_initializer.py:116-124 의 "목표가 충분히 움직였을 때만" 게이트.
            # threshold 0.0 이면 항상 통과한다(우리 기본값).
            if self.goal_change_position_threshold > 0.0:
                import numpy as np
                new_goal = np.array(g[:3], dtype=float)
                if self._previous_goal_position is not None:
                    d = float(np.linalg.norm(self._previous_goal_position - new_goal))
                    if d <= self.goal_change_position_threshold:
                        self.get_logger().warning(
                            f'목표가 {d:.3f} m 밖에 안 움직였다 '
                            f'(<= {self.goal_change_position_threshold}) — 재계획 생략. '
                            '⚠️ 목표 고정 + 장애물 회피 용도라면 이 값을 0.0 으로 둬야 한다.')
                        return None
                self._previous_goal_position = new_goal
            return self.client.pose_constraints(ps)

        self.get_logger().error(f"goal_type:={self.goal_type} 는 없다. joint | pose.")
        return None

    def _arrived(self) -> bool:
        if self.goal_type != 'joint':
            return False
        target = [v * D2R for v in self._current_target_deg()]
        cur = self.current_joints()
        return max(abs(cur[i] - target[i]) for i in range(len(cur))) < self.joint_goal_tol

    def _finish(self, reason: str) -> None:
        """타이머를 멈추고 spin() 을 빠져나가게 한다.

        타이머 콜백은 executor 의 워커 스레드에서 도므로 여기서 예외를 던져도
        main 의 spin() 까지 안 올라간다. rclpy.shutdown() 이라야 spin() 이 리턴한다.
        """
        try:
            self.timer.cancel()
        except Exception:
            pass
        self.get_logger().info(f'{reason} | {self.summary()}')
        if rclpy.ok():
            rclpy.shutdown()

    def _advance(self) -> bool:
        """구간(경유점 또는 최종 목표) 하나를 마쳤을 때. True 면 루프를 끝낸 것이다.

        경유점이 남아 있으면 다음 경유점으로 넘어가고, 다 썼으면 _on_reached() 로 간다.
        """
        if self._use_waypoints():
            self._wp_idx += 1
            if self._wp_idx < self.waypoints:
                self.get_logger().info(
                    f'  └ 경유점 {self._wp_idx}/{self.waypoints} 통과 — 다음 경유점으로')
                return False
        self._wp_idx = 0
        self._first_leg = False
        return self._on_reached()

    def _on_reached(self) -> bool:
        """최종 목표(A 또는 B)에 도달했을 때의 처리. True 면 루프를 끝낸 것이다."""
        if self.pingpong:
            self._target_idx ^= 1
            if self._target_idx == 0:
                self._cycle_count += 1
                self.get_logger().info(f'왕복 {self._cycle_count}회 | {self.summary()}')
                if self.cycles and self._cycle_count >= self.cycles:
                    self._finish('지정 왕복 완료 — 종료')
                    return True
            nxt = 'B' if self._target_idx == 1 else 'A'
            self.get_logger().info(f'도착 → 목표 {nxt} 로 전환')
            return False
        if self.stop_when_arrived:
            self._finish('목표 도착 — 종료')
            return True
        return False

    def on_timer(self) -> None:
        # sequential 모드에서 send_goal 이 블록하는 동안 타이머가 재진입하지 않게 막는다.
        # (원본은 MutuallyExclusive 콜백그룹이라 자동으로 막혔다. 우리는 Reentrant 라 직접 막는다)
        if self.mode == 'sequential' and self._busy:
            return

        # ── 도착 판정: 🔴 모드마다 근거가 달라야 한다 (2026-08-08 실측으로 배운 것) ──
        #   sequential : MoveGroup 결과(SUCCESS)로 판정한다 → 아래 결과 분기의 _on_reached()
        #   preemptive : 결과를 안 기다리므로 /joint_states 실측으로 여기서 판정한다
        #
        # ⚠️ sequential 에 관절공간 판정을 쓰면 안 된다. cuMotion 은 관절 목표를 FK 로 pose 로
        #    바꾼 뒤 IK 를 푸는데(cumotion_planner.py:714-730), **같은 pose 를 만드는 다른 IK 해**
        #    로 갈 수 있다. 그러면 pose 는 맞는데 관절값이 달라 _arrived() 가 영영 False 다.
        #    2026-08-08 실측: 목표 B 에서 정확히 이게 나서 B→A 전환이 안 되고 제자리 궤적만
        #    5.89초씩 무한 반복했다. (A 에서는 우연히 같은 IK 해가 나와 문제가 안 드러났다)
        if self.mode == 'preemptive' and self._arrived():
            if self._advance():
                return

        constraints = self._build_constraints()
        if constraints is None:
            return

        wait = (self.mode == 'sequential')

        # 🔴 선점 실험 스위치 — 새 goal 을 던지기 **전에** 이전 goal 을 취소한다.
        #    ⚠️ send_goal 뒤에 두면 방금 보낸 goal 을 취소하게 된다. 반드시 앞이어야 한다.
        #
        #    2026-08-08 preemptive 실측: 8초 → 30초 → 72초+ 로 갈수록 느려졌다.
        #    두 가설이 구분이 안 된다 —
        #      (a) 취소를 안 해서 goal 이 쌓이고 로봇이 한참 전 궤적을 실행 중이다
        #      (b) 새 goal 이 매번 실행을 abort 시켜 v=0 에서 재출발한다(구조적 한계)
        #    이 스위치를 켜고 시간이 확 줄면 (a), 그대로면 (b) 다.
        if not wait and self.cancel_previous:
            self.client.cancel()

        self._busy = True
        self.stat_goals += 1
        name = 'B' if (self.pingpong and self._target_idx == 1) else 'A'
        self.get_logger().info(f'goal #{self.stat_goals} 전송 (목표 {name})')

        t0 = time.time()
        try:
            result = self.client.send_goal(
                constraints, self.vel_scale, self.acc_scale,
                allowed_planning_time=self.allowed_planning_time, wait=wait,
                replan=self.replan, replan_attempts=self.replan_attempts,
                replan_delay=self.replan_delay)
        finally:
            self._busy = False

        if not wait:
            return   # 선점 모드: 결과는 다음 틱에서 로그로만 확인
        elapsed = time.time() - t0

        if result is None:
            self.stat_fail += 1
            self.get_logger().error('결과 없음(타임아웃)')
            return
        if result == 'REJECTED':
            self.stat_fail += 1
            return

        code = result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            # 🔴 elapsed = 이 goal 이 나가고 실행이 끝날 때까지의 시간.
            #    이 구간 내내 **새 계획이 안 들어간다** = 장애물이 들어와도 못 본다.
            #    예제 방식(plan_only=False)이 동적 회피를 못 하는 이유가 이 숫자다.
            #    2026-08-07 실측: 이미 목표에 있는데도 매번 5.8~6.0초.
            self.get_logger().info(
                f'✅ 목표 {name} 도착 (plan+execute 완료) | '
                f'회피 불가 구간 {elapsed:.2f}초')
            # 🔴 sequential 의 전환은 여기서 한다 — MoveGroup 이 SUCCESS 를 줬다는 것이
            #    "도착"의 유일하게 믿을 수 있는 신호다(위 on_timer 주석의 IK 해 문제).
            if self._advance():
                return
        else:
            self.stat_fail += 1
            hint = ''
            if code == MoveItErrorCodes.PLANNING_FAILED:
                hint = (' → cuMotion 경로면 원인이 안 담긴다(플러그인이 덮어씀). '
                        'T6 로그의 "Motion planning failed wih status:" 를 본다.')
            elif code == MoveItErrorCodes.CONTROL_FAILED:
                hint = (' → 실행 단계 실패. allowed_start_tolerance(0.08) 위반이거나 '
                        'JTC 가 궤적을 거부했다.')
            elif code == MoveItErrorCodes.FAILURE:
                hint = ' → cumotion_planner_node 가 이미 다른 계획 처리 중(planner_busy)일 수 있다.'
            self.get_logger().error(f'❌ 실패: {err_name(code)}{hint}')

    def summary(self) -> str:
        return f'goal {self.stat_goals}회 (실패 {self.stat_fail})'


def main(args=None) -> int:
    rclpy.init(args=args)
    node = GoalSetterReplanNode()

    if not node.client.wait_for_server(timeout=30.0):
        node.destroy_node()
        rclpy.shutdown()
        return 1

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    rc = 0
    try:
        executor.spin()
    except ExternalShutdownException:
        # _finish() 가 rclpy.shutdown() 을 부른 정상 종료 경로다. 이미 요약을 찍었다.
        pass
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('중단 — goal 취소')
        node.client.cancel()
    finally:
        try:
            node.get_logger().info(node.summary())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())

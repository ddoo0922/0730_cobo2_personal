"""The motion primitives the LLM is allowed to call.

Every long motion here is issued asynchronously (``amovel``/``amovej``) and then
polled, instead of using the blocking ``movel``/``movej``. That is the whole
point: a blocking ``movel`` swallows the thread until the arm arrives, so a
"정지" that lands one millisecond after the call would not take effect until the
move it was meant to interrupt had already finished. With the async form the
runner checks ``cancel`` every poll interval, so a stop lands within one poll.

Threading contract
------------------
Doosan's Python API funnels every call through
``rclpy.spin_until_future_complete(DR_init.__dsr__node, ...)``. Two threads
driving that one spin raises "generator already executing". So: **all methods on
DoosanArm must be called from a single thread** (RobotNode's motion worker).
``cancel`` is a plain ``threading.Event`` precisely so that other threads can
interrupt without ever touching the Doosan API. The one hardware-level
interrupt -- the ``motion/move_stop`` service -- is owned by RobotNode and
called on its own executor, never from here.
"""

from dataclasses import dataclass
import math
import threading
import time
from typing import Sequence

# check_motion() status codes, from dsr_msgs2/srv/CheckMotion.srv.
DR_STATE_IDLE = 0
DR_STATE_INIT = 1
DR_STATE_BUSY = 2


class MotionError(RuntimeError):
    """The motion could not be carried out."""


class MotionCancelled(MotionError):
    """The motion was interrupted on purpose. Not a fault."""


@dataclass(frozen=True)
class WorkspaceBounds:
    """The box the tool centre point is allowed to enter, in metres."""

    min_x_m: float = 0.2
    max_x_m: float = 0.9
    min_y_m: float = -0.6
    max_y_m: float = 0.6
    min_z_m: float = 0.02
    max_z_m: float = 0.8

    def validate(self, position_m: Sequence[float], extra_clearance_m: float = 0.0) -> None:
        if len(position_m) != 3:
            raise MotionError("position must contain x, y, z")
        values = []
        for axis, value in zip("xyz", position_m):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise MotionError(f"{axis} must be numeric")
            value = float(value)
            if not math.isfinite(value):
                raise MotionError(f"{axis} must be finite")
            values.append(value)

        x, y, z = values
        for axis, value, low, high in (
            ("x", x, self.min_x_m, self.max_x_m),
            ("y", y, self.min_y_m, self.max_y_m),
            ("z", z, self.min_z_m, self.max_z_m),
        ):
            if not low <= value <= high:
                raise MotionError(
                    f"{axis}={value:.4f}m is outside the safe workspace "
                    f"[{low:.4f}, {high:.4f}]m"
                )
        if z + extra_clearance_m > self.max_z_m:
            raise MotionError("approach height crosses the workspace ceiling")


@dataclass(frozen=True)
class MotionProfile:
    velocity: float = 30.0
    acceleration: float = 30.0
    approach_height_m: float = 0.05
    lift_height_m: float = 0.05
    place_joints: tuple = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
    home_joints: tuple = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
    gripper_force_n: float = 40.0
    grip_settle_s: float = 0.5
    poll_interval_s: float = 0.02
    motion_start_grace_s: float = 0.3
    motion_timeout_s: float = 30.0
    # Named controller presets. `tcp_name` must be the one the wrist hand-eye
    # calibration was recorded with -- `data_recording.py` used "GripperDA_v1" --
    # because every coordinate this system computes is expressed relative to it.
    tool_name: str = "Tool Weight"
    tcp_name: str = "GripperDA_v1"
    # How far above the object the arm parks to let the wrist camera look. Has to
    # clear the gripper's own 190 mm length and still keep the object in frame.
    observe_height_m: float = 0.25
    # A Cartesian service call only confirms that the controller accepted the
    # request. The controller can reject the pose asynchronously (alarm 1206)
    # while check_motion() remains IDLE, so completed moves are checked against
    # the measured pose as well.
    motion_position_tolerance_m: float = 0.01
    motion_orientation_tolerance_deg: float = 3.0
    motion_joint_tolerance_deg: float = 1.0
    # Doosan's Ikin response can say success for an unreachable pose. Run its
    # result back through FK and accept it only inside these tolerances.
    observe_ik_position_tolerance_m: float = 0.005
    observe_ik_orientation_tolerance_deg: float = 2.0
    observe_max_camera_tilt_deg: float = 10.0
    # Before a camera-aware joint move, lift the TCP vertically to this base-Z
    # so the gripper/camera do not sweep across the tabletop during joint yaw.
    observe_transition_z_m: float = 0.28
    # For a far object, rotate the top-down wrist so the side-mounted camera
    # points outward, then keep the camera slightly inward of the object. This
    # leaves the object in view while pulling the TCP into the reachable volume.
    observe_camera_inset_max_m: float = 0.14
    observe_camera_inset_step_m: float = 0.02

    def __post_init__(self):
        if self.approach_height_m <= 0.0:
            raise ValueError("approach_height_m must be positive")
        if self.lift_height_m <= 0.0:
            raise ValueError("lift_height_m must be positive")
        if self.poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s must be positive")
        if self.motion_timeout_s <= 0.0:
            raise ValueError("motion_timeout_s must be positive")
        if self.motion_position_tolerance_m <= 0.0:
            raise ValueError("motion_position_tolerance_m must be positive")
        if self.motion_orientation_tolerance_deg <= 0.0:
            raise ValueError("motion_orientation_tolerance_deg must be positive")
        if self.motion_joint_tolerance_deg <= 0.0:
            raise ValueError("motion_joint_tolerance_deg must be positive")
        if self.observe_ik_position_tolerance_m <= 0.0:
            raise ValueError("observe_ik_position_tolerance_m must be positive")
        if self.observe_ik_orientation_tolerance_deg <= 0.0:
            raise ValueError("observe_ik_orientation_tolerance_deg must be positive")
        if not 0.0 < self.observe_max_camera_tilt_deg < 90.0:
            raise ValueError("observe_max_camera_tilt_deg must be between 0 and 90")
        if self.observe_transition_z_m <= 0.0:
            raise ValueError("observe_transition_z_m must be positive")
        if self.observe_camera_inset_max_m < 0.0:
            raise ValueError("observe_camera_inset_max_m cannot be negative")
        if self.observe_camera_inset_step_m <= 0.0:
            raise ValueError("observe_camera_inset_step_m must be positive")
        if len(self.place_joints) != 6 or len(self.home_joints) != 6:
            raise ValueError("joint targets must have six values")


def _zyz_rotation(angles_deg: Sequence[float]) -> tuple:
    """Return a 3x3 rotation for Doosan's intrinsic ZYZ Euler convention."""

    if len(angles_deg) != 3:
        raise MotionError("ZYZ orientation must contain three angles")
    a, b, c = (math.radians(float(v)) for v in angles_deg)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    return (
        (ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb),
        (sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb),
        (-sb * cc, sb * sc, cb),
    )


def _rotate_vector(rotation, vector: Sequence[float]) -> tuple:
    return tuple(
        sum(float(rotation[row][column]) * float(vector[column]) for column in range(3))
        for row in range(3)
    )


def _rotation_error_deg(first: Sequence[float], second: Sequence[float]) -> float:
    """Geodesic rotation error; unlike Euler subtraction it is safe at B=180°."""

    left = _zyz_rotation(first)
    right = _zyz_rotation(second)
    # trace(R_left^T R_right) is the Frobenius inner product of both matrices.
    trace_relative = sum(
        left[row][column] * right[row][column]
        for row in range(3)
        for column in range(3)
    )
    cosine = max(-1.0, min(1.0, (trace_relative - 1.0) * 0.5))
    return math.degrees(math.acos(cosine))


def _check_cancel(cancel: threading.Event, where: str) -> None:
    if cancel.is_set():
        raise MotionCancelled(f"cancelled during {where}")


class DryRunArm:
    """Validates and times out motions without touching any hardware.

    Keeping the timing roughly realistic matters: the agent's decision loop is
    driven by action-completion events, so an executor that returned instantly
    would never exercise the "user speaks while the arm is moving" path that
    scenarios 2 and 3 are entirely about.
    """

    motion_enabled = False

    def __init__(
        self,
        bounds: WorkspaceBounds,
        profile: MotionProfile,
        logger=None,
        simulated_seconds: float = 1.2,
    ):
        if simulated_seconds <= 0.0:
            raise ValueError("simulated_seconds must be positive")
        self.bounds = bounds
        self.profile = profile
        self.logger = logger
        # Raise this to open a window wide enough to interrupt a motion by
        # hand: a real reach takes several seconds, but an LLM round-trip
        # takes a few too, so a snappy simulation finishes before the user's
        # "그 사과는 집지마" has even been decided on.
        self.simulated_seconds = simulated_seconds

    def _log(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(f"[dry-run] {message}")

    def _sleep(self, cancel: threading.Event, seconds: float, where: str) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            _check_cancel(cancel, where)
            time.sleep(self.profile.poll_interval_s)
        _check_cancel(cancel, where)

    def pick(self, position_m, cancel: threading.Event, on_grasped=None) -> None:
        self.bounds.validate(position_m, self.profile.approach_height_m)
        x, y, z = (float(v) for v in position_m)
        self._log(f"pick at ({x:.4f}, {y:.4f}, {z:.4f}) m")
        self._sleep(cancel, self.simulated_seconds, "approach")
        if on_grasped is not None:
            on_grasped()
        self._sleep(cancel, self.profile.grip_settle_s, "grip")
        self._sleep(cancel, self.simulated_seconds * 0.5, "lift")

    def observe(self, position_m, cancel: threading.Event) -> None:
        observe_z = float(position_m[2]) + self.profile.observe_height_m
        self.bounds.validate((float(position_m[0]), float(position_m[1]), observe_z))
        self._log(
            f"observe from ({float(position_m[0]):.4f}, {float(position_m[1]):.4f}, "
            f"{observe_z:.4f}) m"
        )
        self._sleep(cancel, self.simulated_seconds, "observe")

    def pick_at_posx(
        self, target_posx, pregrasp_posx, cancel: threading.Event, on_grasped=None
    ) -> None:
        for name, pose in (("pregrasp", pregrasp_posx), ("grasp", target_posx)):
            if len(pose) != 6:
                raise MotionError(f"{name} posx must have six values")
            self.bounds.validate([float(v) / 1000.0 for v in pose[:3]])
        self._log(
            f"grasp at ({target_posx[0]:.1f}, {target_posx[1]:.1f}, "
            f"{target_posx[2]:.1f}) mm ZYZ ({target_posx[3]:.1f}, "
            f"{target_posx[4]:.1f}, {target_posx[5]:.1f})"
        )
        self._sleep(cancel, self.simulated_seconds, "grasp approach")
        self._sleep(cancel, self.simulated_seconds * 0.5, "grasp descent")
        if on_grasped is not None:
            on_grasped()
        self._sleep(cancel, self.profile.grip_settle_s, "grip")
        self._sleep(cancel, self.simulated_seconds * 0.5, "retreat and lift")

    def place(self, cancel: threading.Event) -> None:
        self._log("place into the basket")
        self._sleep(cancel, self.simulated_seconds, "place")

    def release(self, cancel: threading.Event) -> None:
        del cancel
        self._log("open the gripper in place")

    def go_home(self, cancel: threading.Event) -> None:
        self._log("return home")
        self._sleep(cancel, self.simulated_seconds, "home")

    def close(self) -> None:
        return None


class DoosanArm:
    """Real M0609 + RG2 motion. Single-threaded, see the module docstring."""

    motion_enabled = True

    def __init__(
        self,
        robot_id: str,
        robot_model: str,
        bounds: WorkspaceBounds,
        profile: MotionProfile,
        gripper,
        logger=None,
        camera_offset_gripper_mm=None,
        camera_axis_gripper=None,
    ):
        import rclpy
        import DR_init

        # Plain `DR_init.__dsr__id = ...` would silently become
        # `DR_init._DoosanArm__dsr__id = ...` here: any dunder-shaped identifier
        # (leading `__`, no trailing `__`) written inside a class body is
        # name-mangled by Python at compile time, even as an attribute target on
        # an external module. DSR_ROBOT2.py reads the unmangled
        # `DR_init.__dsr__node` at import time, so the mangled write would leave
        # it at its default `None` and crash with "AttributeError: 'NoneType'
        # object has no attribute 'create_client'". setattr() takes the name as
        # a string, so it is never mangled.
        setattr(DR_init, "__dsr__id", robot_id)
        setattr(DR_init, "__dsr__model", robot_model)
        self.node = rclpy.create_node("vla_motion_driver", namespace=robot_id)
        setattr(DR_init, "__dsr__node", self.node)

        from DSR_ROBOT2 import (
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            amovej,
            amovel,
            check_motion,
            fkin,
            get_current_posj,
            get_current_posx,
            get_current_solution_space,
            ikin,
            set_tcp,
            set_tool,
        )
        from DR_common2 import posj, posx

        self.absolute_mode = DR_MV_MOD_ABS
        self.relative_mode = DR_MV_MOD_REL
        self.amovej = amovej
        self.amovel = amovel
        self.check_motion = check_motion
        self.fkin = fkin
        self.get_current_posj = get_current_posj
        self.get_current_posx = get_current_posx
        self.get_current_solution_space = get_current_solution_space
        self.ikin = ikin
        self.posj = posj
        self.posx = posx

        self.bounds = bounds
        self.profile = profile
        self.gripper = gripper
        if camera_offset_gripper_mm is None:
            self.camera_offset_gripper_mm = None
        else:
            offset = tuple(float(v) for v in camera_offset_gripper_mm)
            if len(offset) != 3 or not all(math.isfinite(v) for v in offset):
                raise MotionError("camera-to-gripper translation must be three finite values")
            self.camera_offset_gripper_mm = offset
        if camera_axis_gripper is None:
            self.camera_axis_gripper = None
        else:
            axis = tuple(float(v) for v in camera_axis_gripper)
            norm = math.sqrt(sum(value * value for value in axis))
            if (
                len(axis) != 3
                or not all(math.isfinite(value) for value in axis)
                or norm <= 0.0
            ):
                raise MotionError("camera optical axis must be three finite values")
            self.camera_axis_gripper = tuple(value / norm for value in axis)
        # Assigned before anything calls _log(): the TCP setup below logs, and
        # doing that before this line raised AttributeError on startup.
        self.logger = logger
        self.place_joints = posj([float(v) for v in profile.place_joints])
        self.home_joints = posj([float(v) for v in profile.home_joints])

        # The TCP has to be set here, and it has to be the same one the hand-eye
        # calibration was measured with. `get_current_posx()` reports whichever
        # TCP the controller currently holds, so a different one silently shifts
        # the origin of every coordinate this system computes -- the wrist
        # camera's grasps most of all, since their whole chain hangs off that
        # pose. Leaving it unset means inheriting whatever the last program left
        # behind, which is the same bug with no way to notice it.
        if profile.tool_name:
            set_tool(profile.tool_name)
        if profile.tcp_name:
            set_tcp(profile.tcp_name)
        self._log(
            f"tool={profile.tool_name!r} tcp={profile.tcp_name!r} "
            "(must match the hand-eye calibration)"
        )

    # ------------------------------------------------------------- internals

    def _log(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(message)

    def _warn(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warning(message)

    def _current_pose(self) -> list:
        current = self.get_current_posx()
        if current is None or isinstance(current, (int, float)) or current[0] is None:
            raise MotionError("get_current_posx returned no pose")
        values = [float(v) for v in current[0][:6]]
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise MotionError("get_current_posx returned an invalid pose")
        return values

    def _current_orientation(self) -> list:
        return self._current_pose()[3:]

    def _current_joints(self) -> list:
        current = self.get_current_posj()
        if current is None or isinstance(current, (int, float)):
            raise MotionError("get_current_posj returned no pose")
        values = [float(v) for v in current[:6]]
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise MotionError("get_current_posj returned an invalid pose")
        return values

    @staticmethod
    def _pose_errors(target: Sequence[float], actual: Sequence[float]) -> tuple:
        target_values = [float(v) for v in target]
        actual_values = [float(v) for v in actual]
        if len(target_values) < 6 or len(actual_values) < 6:
            raise MotionError("a Cartesian pose must contain six values")
        position_error_mm = math.dist(target_values[:3], actual_values[:3])
        orientation_error_deg = _rotation_error_deg(
            target_values[3:6], actual_values[3:6]
        )
        return position_error_mm, orientation_error_deg

    def _verify_pose(self, target: Sequence[float], where: str) -> list:
        actual = self._current_pose()
        position_error_mm, orientation_error_deg = self._pose_errors(target, actual)
        position_limit_mm = self.profile.motion_position_tolerance_m * 1000.0
        orientation_limit_deg = self.profile.motion_orientation_tolerance_deg
        if (
            position_error_mm > position_limit_mm
            or orientation_error_deg > orientation_limit_deg
        ):
            raise MotionError(
                f"{where} target was not reached: position error "
                f"{position_error_mm:.1f}mm (limit {position_limit_mm:.1f}mm), "
                f"orientation error {orientation_error_deg:.1f}deg "
                f"(limit {orientation_limit_deg:.1f}deg); actual TCP "
                f"({actual[0]:.1f}, {actual[1]:.1f}, {actual[2]:.1f})mm"
            )
        return actual

    def _verify_position(self, target_xyz: Sequence[float], where: str) -> list:
        actual = self._current_pose()
        error_mm = math.dist([float(v) for v in target_xyz[:3]], actual[:3])
        limit_mm = self.profile.motion_position_tolerance_m * 1000.0
        if error_mm > limit_mm:
            raise MotionError(
                f"{where} target was not reached: position error {error_mm:.1f}mm "
                f"(limit {limit_mm:.1f}mm); actual TCP "
                f"({actual[0]:.1f}, {actual[1]:.1f}, {actual[2]:.1f})mm"
            )
        return actual

    def _verify_joints(self, target: Sequence[float], where: str) -> list:
        target_values = [float(v) for v in target[:6]]
        actual = self._current_joints()
        error_deg = max(abs(want - got) for want, got in zip(target_values, actual))
        limit_deg = self.profile.motion_joint_tolerance_deg
        if error_deg > limit_deg:
            raise MotionError(
                f"{where} joint target was not reached: max error "
                f"{error_deg:.1f}deg (limit {limit_deg:.1f}deg)"
            )
        return actual

    def _ik_solution(self, target: Sequence[float]):
        """Return a verified joint solution, or None when the pose is unreachable."""

        try:
            solution_space = int(self.get_current_solution_space())
            if not 0 <= solution_space <= 7:
                raise ValueError(f"invalid solution space {solution_space}")
            joints = self.ikin(self.posx([float(v) for v in target[:6]]), solution_space)
            if joints is None or isinstance(joints, (int, float)):
                raise ValueError("Ikin returned no joint pose")
            joint_values = [float(v) for v in joints[:6]]
            if len(joint_values) != 6 or not all(math.isfinite(v) for v in joint_values):
                raise ValueError("Ikin returned an invalid joint pose")
            round_trip = self.fkin(joint_values)
            if round_trip is None or isinstance(round_trip, (int, float)):
                raise ValueError("Fkin returned no Cartesian pose")
            round_trip_values = [float(v) for v in round_trip[:6]]
            if len(round_trip_values) != 6 or not all(
                math.isfinite(v) for v in round_trip_values
            ):
                raise ValueError("Fkin returned an invalid Cartesian pose")
        except Exception as exc:
            raise MotionError(f"could not validate observation pose with IK/FK: {exc}") from exc

        position_error_mm, orientation_error_deg = self._pose_errors(
            target, round_trip_values
        )
        if (
            position_error_mm
            > self.profile.observe_ik_position_tolerance_m * 1000.0
            or orientation_error_deg
            > self.profile.observe_ik_orientation_tolerance_deg
        ):
            return None
        return joint_values

    def _wait_for_motion(self, cancel: threading.Event, where: str) -> bool:
        """Block until the controller reports IDLE, or the caller cancels.

        Two phases, because ``check_motion()`` polled immediately after an
        async move can still read the *pre-motion* IDLE. Returning on that
        would report a move as complete before the arm had begun to move.
        """
        saw_active = False
        grace_deadline = time.monotonic() + self.profile.motion_start_grace_s
        while time.monotonic() < grace_deadline:
            _check_cancel(cancel, where)
            status = self.check_motion()
            if status is None or status < 0:
                raise MotionError(f"check_motion failed during {where}")
            if status != DR_STATE_IDLE:
                saw_active = True
                break
            time.sleep(self.profile.poll_interval_s)

        deadline = time.monotonic() + self.profile.motion_timeout_s
        while True:
            _check_cancel(cancel, where)
            status = self.check_motion()
            if status == DR_STATE_IDLE:
                return saw_active
            if status is None or status < 0:
                raise MotionError(f"check_motion failed during {where}")
            saw_active = True
            if time.monotonic() > deadline:
                raise MotionError(
                    f"motion timed out after {self.profile.motion_timeout_s:.1f}s "
                    f"during {where}"
                )
            time.sleep(self.profile.poll_interval_s)

    def _movel(self, target, cancel: threading.Event, where: str, relative=False) -> None:
        _check_cancel(cancel, where)
        target_values = [float(v) for v in target[:6]]
        if len(target_values) != 6:
            raise MotionError(f"{where} target must contain six values")
        start_pose = self._current_pose() if relative else None
        # A stop may arrive while the synchronous pose service above is in
        # flight.  Recheck before submitting the async move so an idle MoveStop
        # cannot be followed by a late motion command.
        _check_cancel(cancel, where)
        result = self.amovel(
            target,
            vel=self.profile.velocity,
            acc=self.profile.acceleration,
            mod=self.relative_mode if relative else self.absolute_mode,
        )
        if result is not None and result < 0:
            raise MotionError(f"amovel rejected during {where}")
        self._wait_for_motion(cancel, where)
        if relative:
            expected_xyz = [
                start_pose[index] + target_values[index] for index in range(3)
            ]
            self._verify_position(expected_xyz, where)
        else:
            self._verify_pose(target_values, where)

    def _movej(self, target, cancel: threading.Event, where: str) -> None:
        _check_cancel(cancel, where)
        result = self.amovej(
            target,
            vel=self.profile.velocity,
            acc=self.profile.acceleration,
        )
        if result is not None and result < 0:
            raise MotionError(f"amovej rejected during {where}")
        self._wait_for_motion(cancel, where)
        self._verify_joints(target, where)

    def _grip(self, close: bool, cancel: threading.Event, where: str) -> None:
        _check_cancel(cancel, where)
        if close:
            self.gripper.close(self.profile.gripper_force_n)
        else:
            self.gripper.open(self.profile.gripper_force_n)
        self.gripper.wait_until_idle()

    def _move_to_observation_pose(
        self,
        target_pose: Sequence[float],
        joint_solution: Sequence[float],
        cancel: threading.Event,
        where: str,
    ) -> None:
        """Reach a verified observation endpoint without a singular Cartesian path.

        ``_ik_solution`` validates a concrete joint endpoint.  Executing that
        same endpoint keeps validation and execution consistent; sending the
        exact Cartesian pose instead caused the controller to reject a valid
        3.7 mm boundary approximation with alarm 1206.  The vertical linear
        lift keeps the camera/gripper clear before the joint interpolation.
        """

        transition_z_mm = min(
            float(target_pose[2]), self.profile.observe_transition_z_m * 1000.0
        )
        current_pose = self._current_pose()
        if (
            current_pose[2]
            < transition_z_mm
            - self.profile.motion_position_tolerance_m * 1000.0
        ):
            transition_pose = [
                current_pose[0],
                current_pose[1],
                transition_z_mm,
                *current_pose[3:],
            ]
            self.bounds.validate(
                [value / 1000.0 for value in transition_pose[:3]]
            )
            if self._ik_solution(transition_pose) is None:
                raise MotionError(
                    f"{where} needs a vertical safety lift, but its endpoint "
                    "is unreachable"
                )
            self._log(
                f"raise TCP vertically to {transition_z_mm:.1f}mm before {where}"
            )
            self._movel(
                self.posx(transition_pose),
                cancel,
                "observation safety lift",
            )

        self._movej(self.posj(joint_solution), cancel, where)
        self._verify_pose(target_pose, where)

    # ------------------------------------------------------------ primitives

    def pick(self, position_m, cancel: threading.Event, on_grasped=None) -> None:
        """Approach from above, close on the object, and lift clear.

        ``on_grasped`` fires the moment the fingers have closed, before the
        settle and the lift. The caller uses it to record that the arm is now
        physically holding something -- a stop landing during the lift must not
        leave the reported state saying the gripper is empty when it is not.
        """
        self.bounds.validate(position_m, self.profile.approach_height_m)
        x_mm, y_mm, z_mm = (float(v) * 1000.0 for v in position_m)
        approach_mm = self.profile.approach_height_m * 1000.0
        lift_mm = self.profile.lift_height_m * 1000.0

        orientation = self._current_orientation()
        pregrasp = self.posx([x_mm, y_mm, z_mm + approach_mm, *orientation])
        grasp = self.posx([x_mm, y_mm, z_mm, *orientation])
        self._log(f"pick at ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm")

        self._grip(close=False, cancel=cancel, where="gripper open")
        self._movel(pregrasp, cancel, "approach")
        self._movel(grasp, cancel, "descent")
        self._grip(close=True, cancel=cancel, where="grasp")
        if on_grasped is not None:
            on_grasped()

        # A settle that ignored cancel would keep the arm committed for half a
        # second after a stop; this one gives the fingers the same time but
        # still bails out.
        settle_deadline = time.monotonic() + self.profile.grip_settle_s
        while time.monotonic() < settle_deadline:
            _check_cancel(cancel, "grip settle")
            time.sleep(self.profile.poll_interval_s)

        self._movel(
            self.posx(0.0, 0.0, lift_mm, 0.0, 0.0, 0.0), cancel, "lift", relative=True
        )

    def observe(self, position_m, cancel: threading.Event) -> None:
        """Park above the object so the wrist camera can see it.

        Try the old TCP-over-object pose first. If it is outside the arm's real
        kinematic volume, use the calibrated lateral camera offset to keep the
        camera on the object while pulling the TCP toward the robot. Every
        candidate is IK->FK checked before motion, and the measured final pose
        is checked again after motion.
        """
        x_mm, y_mm, _z_mm = (float(v) * 1000.0 for v in position_m)
        observe_z_mm = float(position_m[2]) * 1000.0 + self.profile.observe_height_m * 1000.0
        self.bounds.validate(
            (float(position_m[0]), float(position_m[1]), observe_z_mm / 1000.0)
        )
        current_pose = self._current_pose()
        orientation = current_pose[3:]
        if self.camera_axis_gripper is not None:
            camera_axis_base = _rotate_vector(
                _zyz_rotation(orientation), self.camera_axis_gripper
            )
            down_alignment = max(-1.0, min(1.0, -camera_axis_base[2]))
            camera_tilt_deg = math.degrees(math.acos(down_alignment))
            if camera_tilt_deg > self.profile.observe_max_camera_tilt_deg:
                raise MotionError(
                    f"wrist camera is tilted {camera_tilt_deg:.1f}deg from down "
                    f"(limit {self.profile.observe_max_camera_tilt_deg:.1f}deg); "
                    "return home before wrist observation"
                )
        self._grip(close=False, cancel=cancel, where="gripper open")
        direct_pose = [x_mm, y_mm, observe_z_mm, *orientation]
        direct_solution = self._ik_solution(direct_pose)
        if direct_solution is not None:
            self._log(
                f"observe from ({x_mm:.1f}, {y_mm:.1f}, {observe_z_mm:.1f}) mm"
            )
            self._move_to_observation_pose(
                direct_pose, direct_solution, cancel, "observe"
            )
            return

        if self.camera_offset_gripper_mm is None:
            raise MotionError(
                "direct observation pose is unreachable and the wrist camera "
                "calibration is unavailable for an alternate pose"
            )

        radius_mm = math.hypot(x_mm, y_mm)
        if radius_mm <= 1.0:
            raise MotionError("cannot aim the wrist camera at an object on the base axis")
        radial_angle = math.atan2(y_mm, x_mm)
        camera_offset_base = _rotate_vector(
            _zyz_rotation(orientation), self.camera_offset_gripper_mm
        )
        lateral_offset_mm = math.hypot(
            camera_offset_base[0], camera_offset_base[1]
        )
        if lateral_offset_mm <= 1.0:
            raise MotionError(
                "direct observation pose is unreachable and the wrist camera "
                "has no usable lateral calibration offset"
            )

        # Rotating only the first Z of ZYZ is a base-frame yaw. The optical
        # axis stays down, while the side-mounted camera translation rotates to
        # point radially outward. An inward camera inset then pulls the TCP even
        # farther into the arm's reachable volume; the object remains well
        # inside the D435's field of view at the configured 140 mm maximum.
        offset_angle = math.atan2(camera_offset_base[1], camera_offset_base[0])
        yaw_delta = radial_angle - offset_angle
        alternate_orientation = [
            ((orientation[0] + math.degrees(yaw_delta) + 180.0) % 360.0) - 180.0,
            orientation[1],
            orientation[2],
        ]
        outward_offset = _rotate_vector(
            _zyz_rotation(alternate_orientation), self.camera_offset_gripper_mm
        )
        radial_x, radial_y = math.cos(radial_angle), math.sin(radial_angle)

        maximum_inset = self.profile.observe_camera_inset_max_m
        inset_step = self.profile.observe_camera_inset_step_m
        candidate_count = int(math.ceil(maximum_inset / inset_step)) + 1
        for index in range(candidate_count):
            inset_m = min(float(index) * inset_step, maximum_inset)
            inset_mm = inset_m * 1000.0
            camera_x_mm = x_mm - inset_mm * radial_x
            camera_y_mm = y_mm - inset_mm * radial_y
            candidate = [
                camera_x_mm - outward_offset[0],
                camera_y_mm - outward_offset[1],
                observe_z_mm,
                *alternate_orientation,
            ]
            try:
                self.bounds.validate([value / 1000.0 for value in candidate[:3]])
            except MotionError:
                continue
            joint_solution = self._ik_solution(candidate)
            if joint_solution is None:
                continue

            self._warn(
                "direct observation pose is unreachable; using camera-aware "
                f"pose ({candidate[0]:.1f}, {candidate[1]:.1f}, "
                f"{candidate[2]:.1f})mm with {inset_mm:.0f}mm inward view offset"
            )
            self._move_to_observation_pose(
                candidate, joint_solution, cancel, "camera-aware observe"
            )
            return

        raise MotionError(
            "no reachable wrist observation pose was found for "
            f"({x_mm:.1f}, {y_mm:.1f}, {observe_z_mm:.1f})mm"
        )

    def pick_at_posx(
        self,
        target_posx,
        pregrasp_posx,
        cancel: threading.Event,
        on_grasped=None,
    ) -> None:
        """Execute a 6-DOF grasp the wrist camera planned.

        Unlike ``pick``, the orientation comes from the plan rather than from
        wherever the wrist happens to be pointing -- that is the whole reason for
        running GraspGenX. Both poses are already contact-point ``posx`` in mm
        and ZYZ degrees; this method does no geometry, so a frame mistake cannot
        be introduced here.
        """
        for name, pose in (("pregrasp", pregrasp_posx), ("grasp", target_posx)):
            if len(pose) != 6:
                raise MotionError(f"{name} posx must have six values")
            self.bounds.validate([float(v) / 1000.0 for v in pose[:3]])

        self._log(
            f"grasp at ({target_posx[0]:.1f}, {target_posx[1]:.1f}, "
            f"{target_posx[2]:.1f}) mm, ZYZ ({target_posx[3]:.1f}, "
            f"{target_posx[4]:.1f}, {target_posx[5]:.1f})"
        )
        self._grip(close=False, cancel=cancel, where="gripper open")
        self._movel(self.posx(list(pregrasp_posx)), cancel, "grasp approach")
        self._movel(self.posx(list(target_posx)), cancel, "grasp descent")
        self._grip(close=True, cancel=cancel, where="grasp")
        if on_grasped is not None:
            on_grasped()

        settle_deadline = time.monotonic() + self.profile.grip_settle_s
        while time.monotonic() < settle_deadline:
            _check_cancel(cancel, "grip settle")
            time.sleep(self.profile.poll_interval_s)

        # Retreat back along the approach axis, i.e. to the pregrasp pose, before
        # lifting: pulling straight up out of a tilted grasp drags the fingers
        # sideways through whatever was beside the object.
        self._movel(self.posx(list(pregrasp_posx)), cancel, "grasp retreat")
        self._movel(
            self.posx(0.0, 0.0, self.profile.lift_height_m * 1000.0, 0.0, 0.0, 0.0),
            cancel,
            "lift",
            relative=True,
        )

    def place(self, cancel: threading.Event) -> None:
        """Carry the held object to the basket pose and let go."""
        self._log("place into the basket")
        self._movej(self.place_joints, cancel, "carry to basket")
        self._grip(close=False, cancel=cancel, where="release into basket")

    def release(self, cancel: threading.Event) -> None:
        """Open the gripper where the arm is now, without moving it."""
        self._log("open the gripper in place")
        self._grip(close=False, cancel=cancel, where="release")

    def go_home(self, cancel: threading.Event) -> None:
        self._log("return home")
        self._movej(self.home_joints, cancel, "home")

    def close(self) -> None:
        try:
            self.gripper.close_connection()
        finally:
            self.node.destroy_node()

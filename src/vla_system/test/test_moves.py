"""Cancellation and workspace limits, without a robot in the room."""

import math
import threading
import unittest

from vla_system.robot.moves import (
    DoosanArm,
    DryRunArm,
    MotionCancelled,
    MotionError,
    MotionProfile,
    WorkspaceBounds,
    _rotate_vector,
    _rotation_error_deg,
    _zyz_rotation,
)


class WorkspaceBoundsTest(unittest.TestCase):
    def setUp(self):
        self.bounds = WorkspaceBounds()

    def test_accepts_a_point_inside_the_box(self):
        self.bounds.validate((0.42, -0.18, 0.05))

    def test_rejects_each_axis_outside_the_box(self):
        for position in [(0.1, 0.0, 0.05), (0.5, 0.9, 0.05), (0.5, 0.0, 1.2)]:
            with self.assertRaises(MotionError):
                self.bounds.validate(position)

    def test_rejects_non_finite_coordinates(self):
        for bad in [float("nan"), float("inf")]:
            with self.assertRaises(MotionError):
                self.bounds.validate((0.5, 0.0, bad))

    def test_rejects_booleans_masquerading_as_numbers(self):
        with self.assertRaises(MotionError):
            self.bounds.validate((0.5, 0.0, True))

    def test_rejects_the_wrong_number_of_coordinates(self):
        with self.assertRaises(MotionError):
            self.bounds.validate((0.5, 0.0))

    def test_approach_clearance_must_also_fit(self):
        """Reaching down is checked; so is the hover above the object."""
        self.bounds.validate((0.5, 0.0, 0.75), extra_clearance_m=0.05)
        with self.assertRaises(MotionError):
            self.bounds.validate((0.5, 0.0, 0.78), extra_clearance_m=0.05)


class MotionProfileTest(unittest.TestCase):
    def test_rejects_a_non_positive_approach_height(self):
        with self.assertRaises(ValueError):
            MotionProfile(approach_height_m=0.0)

    def test_rejects_joint_targets_of_the_wrong_length(self):
        with self.assertRaises(ValueError):
            MotionProfile(place_joints=(0.0, 0.0, 90.0))


class DryRunArmTest(unittest.TestCase):
    def setUp(self):
        self.profile = MotionProfile(poll_interval_s=0.001, grip_settle_s=0.01)
        self.arm = DryRunArm(WorkspaceBounds(), self.profile)
        self.arm.simulated_seconds = 0.02

    def test_a_valid_pick_runs_to_completion(self):
        self.arm.pick((0.42, -0.18, 0.05), threading.Event())

    def test_an_out_of_bounds_pick_is_refused_before_moving(self):
        with self.assertRaises(MotionError):
            self.arm.pick((5.0, 0.0, 0.05), threading.Event())

    def test_a_cancel_set_up_front_stops_the_pick(self):
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(MotionCancelled):
            self.arm.pick((0.42, -0.18, 0.05), cancel)

    def test_a_cancel_landing_mid_motion_stops_the_pick(self):
        cancel = threading.Event()
        threading.Timer(0.01, cancel.set).start()
        self.arm.simulated_seconds = 1.0
        with self.assertRaises(MotionCancelled):
            self.arm.pick((0.42, -0.18, 0.05), cancel)

    def test_grasp_is_reported_before_the_lift(self):
        """A stop during the lift must not leave the state saying "empty"."""
        events = []
        self.arm.pick(
            (0.42, -0.18, 0.05), threading.Event(), on_grasped=lambda: events.append("held")
        )
        self.assertEqual(events, ["held"])

    def test_cancel_during_the_lift_still_reports_the_grasp(self):
        cancel = threading.Event()
        events = []

        def on_grasped():
            events.append("held")
            cancel.set()

        with self.assertRaises(MotionCancelled):
            self.arm.pick((0.42, -0.18, 0.05), cancel, on_grasped=on_grasped)
        self.assertEqual(events, ["held"])


class DoosanCompletionTest(unittest.TestCase):
    """Controller service acceptance must not masquerade as completed motion."""

    def make_arm(self, actual_pose):
        arm = DoosanArm.__new__(DoosanArm)
        arm.profile = MotionProfile(
            poll_interval_s=0.0001,
            motion_start_grace_s=0.001,
            motion_timeout_s=0.02,
        )
        arm.amovel = lambda *args, **kwargs: 0
        arm.absolute_mode = 0
        arm.relative_mode = 1
        arm.check_motion = lambda: 0
        arm.get_current_posx = lambda: (list(actual_pose), 2)
        arm.logger = None
        return arm

    def test_idle_forever_and_far_tcp_is_a_rejected_move(self):
        arm = self.make_arm([367.2, 3.2, 182.8, 90.04, 179.98, 90.52])
        target = [559.5, 52.6, 399.9, 93.35, 179.98, 93.82]
        with self.assertRaisesRegex(MotionError, "target was not reached"):
            arm._movel(target, threading.Event(), "observe")

    def test_idle_forever_is_allowed_when_tcp_is_already_at_target(self):
        target = [559.5, 52.6, 399.9, 93.35, 179.98, 93.82]
        arm = self.make_arm(target)
        arm._movel(target, threading.Event(), "observe")

    def test_busy_then_idle_still_requires_the_final_target(self):
        target = [559.5, 52.6, 399.9, 93.35, 179.98, 93.82]
        arm = self.make_arm(target)
        statuses = iter([1, 2, 0])
        arm.check_motion = lambda: next(statuses, 0)
        arm._movel(target, threading.Event(), "observe")

    def test_busy_then_idle_and_far_tcp_is_rejected(self):
        arm = self.make_arm([367.2, 3.2, 182.8, 90.04, 179.98, 90.52])
        statuses = iter([1, 2, 0])
        arm.check_motion = lambda: next(statuses, 0)
        target = [559.5, 52.6, 399.9, 93.35, 179.98, 93.82]
        with self.assertRaisesRegex(MotionError, "target was not reached"):
            arm._movel(target, threading.Event(), "observe")

    def test_cancel_during_start_pose_read_prevents_motion_submission(self):
        arm = self.make_arm([367.2, 3.2, 182.8, 90.04, 179.98, 90.52])
        cancel = threading.Event()

        def read_then_cancel():
            cancel.set()
            return ([367.2, 3.2, 182.8, 90.04, 179.98, 90.52], 2)

        arm.get_current_posx = read_then_cancel
        arm.amovel = lambda *args, **kwargs: self.fail("motion must not be submitted")
        with self.assertRaises(MotionCancelled):
            arm._movel(
                [0.0, 0.0, 50.0, 0.0, 0.0, 0.0],
                cancel,
                "lift",
                relative=True,
            )

    def test_rejected_relative_lift_is_not_treated_as_complete(self):
        actual = [400.0, 0.0, 100.0, 90.0, 180.0, 90.0]
        arm = self.make_arm(actual)
        delta = [0.0, 0.0, 50.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(MotionError, "target was not reached"):
            arm._movel(delta, threading.Event(), "lift", relative=True)

    def test_rejected_joint_move_is_not_treated_as_complete(self):
        arm = self.make_arm([0.0] * 6)
        arm.amovej = lambda *args, **kwargs: 0
        arm.get_current_posj = lambda: [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        target = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        with self.assertRaisesRegex(MotionError, "joint target was not reached"):
            arm._movej(target, threading.Event(), "camera-aware observe")

    def test_equivalent_zyz_at_the_180_degree_singularity_is_not_rejected(self):
        error = _rotation_error_deg(
            [90.04, 179.98, 90.52],
            [119.59, -179.97, 120.54],
        )
        self.assertLess(error, 1.0)

    def test_ik_success_is_rejected_when_fk_does_not_round_trip(self):
        arm = self.make_arm([0.0] * 6)
        arm.posx = lambda values: list(values)
        arm.get_current_solution_space = lambda: 2
        arm.ikin = lambda target, solution: [4.7, 41.9, 0.0, 0.0, 138.1, 5.2]
        arm.fkin = lambda joints: [518.3, 47.2, 352.0, 94.7, -179.7, 95.5]
        target = [559.5, 52.6, 399.9, 93.35, 179.98, 93.82]
        self.assertIsNone(arm._ik_solution(target))

    def test_unreachable_direct_observe_uses_camera_offset_pose(self):
        arm = self.make_arm([367.2, 3.2, 300.0, 90.04, 179.98, 90.52])
        arm.bounds = WorkspaceBounds()
        arm.camera_offset_gripper_mm = (32.09, 77.77, -221.77)
        arm.camera_axis_gripper = None
        arm.posj = lambda values: list(values)
        arm._grip = lambda **kwargs: None
        ik_calls = []

        def solution(target):
            ik_calls.append(list(target))
            return None if len(ik_calls) == 1 else [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

        arm._ik_solution = solution
        moved_joints = []
        verified_poses = []
        arm._movej = lambda target, cancel, where: moved_joints.append(list(target))
        arm._verify_pose = lambda target, where: verified_poses.append(list(target))
        arm._warn = lambda message: None

        arm.observe((0.64, -0.14, 0.15), threading.Event())

        self.assertEqual(moved_joints, [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]])
        candidate = verified_poses[0]
        camera_offset_base = _rotate_vector(
            _zyz_rotation(candidate[3:]), arm.camera_offset_gripper_mm
        )
        # The first alternate has zero inward inset, so its camera origin is
        # centred over the webcam target even though its TCP is closer to base.
        self.assertAlmostEqual(candidate[0] + camera_offset_base[0], 640.0, places=5)
        self.assertAlmostEqual(candidate[1] + camera_offset_base[1], -140.0, places=5)
        self.assertLess(math.hypot(candidate[0], candidate[1]), math.hypot(640.0, -140.0))

    def test_reachable_direct_observe_executes_the_verified_joint_solution(self):
        arm = self.make_arm([367.2, 3.2, 300.0, 90.04, 179.98, 90.52])
        arm.bounds = WorkspaceBounds()
        arm.camera_offset_gripper_mm = (32.09, 77.77, -221.77)
        arm.camera_axis_gripper = None
        arm.posx = lambda values: list(values)
        arm.posj = lambda values: list(values)
        arm._grip = lambda **kwargs: None
        direct_solution = [0.0, 30.0, 0.0, 0.0, 140.0, 0.0]
        arm._ik_solution = lambda target: direct_solution
        arm._movel = lambda *args: self.fail("direct observe must not use amovel")
        moves = []
        arm._movej = lambda target, cancel, where: moves.append((list(target), where))
        verified = []
        arm._verify_pose = lambda target, where: verified.append((list(target), where))
        arm._warn = lambda message: None

        arm.observe((0.4361, -0.1702, 0.1491), threading.Event())

        self.assertEqual(moves, [(direct_solution, "observe")])
        self.assertEqual(verified[0][1], "observe")

    def test_wrist_observation_rejects_a_camera_that_is_not_looking_down(self):
        arm = self.make_arm([367.2, 3.2, 300.0, 0.0, 0.0, 0.0])
        arm.bounds = WorkspaceBounds()
        arm.camera_offset_gripper_mm = (32.09, 77.77, -221.77)
        arm.camera_axis_gripper = (0.0, 0.0, 1.0)
        arm._grip = lambda **kwargs: self.fail("gripper must not move")
        with self.assertRaisesRegex(MotionError, "camera is tilted"):
            arm.observe((0.64, -0.14, 0.15), threading.Event())

    def test_camera_aware_joint_move_is_preceded_by_a_vertical_safety_lift(self):
        arm = self.make_arm([367.2, 3.2, 182.8, 90.04, 179.98, 90.52])
        arm.bounds = WorkspaceBounds()
        arm.camera_offset_gripper_mm = (32.09, 77.77, -221.77)
        arm.camera_axis_gripper = None
        arm.posx = lambda values: list(values)
        arm.posj = lambda values: list(values)
        arm._grip = lambda **kwargs: None
        calls = []
        ik_count = 0

        def solution(target):
            nonlocal ik_count
            ik_count += 1
            return None if ik_count == 1 else [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

        arm._ik_solution = solution
        arm._movel = lambda target, cancel, where: calls.append(
            ("linear", list(target), where)
        )
        arm._movej = lambda target, cancel, where: calls.append(
            ("joint", list(target), where)
        )
        arm._verify_pose = lambda target, where: None
        arm._warn = lambda message: None
        arm._log = lambda message: None

        arm.observe((0.64, -0.14, 0.15), threading.Event())

        self.assertEqual([call[0] for call in calls], ["linear", "joint"])
        self.assertAlmostEqual(calls[0][1][0], 367.2)
        self.assertAlmostEqual(calls[0][1][1], 3.2)
        self.assertAlmostEqual(calls[0][1][2], 280.0)

if __name__ == "__main__":
    unittest.main()

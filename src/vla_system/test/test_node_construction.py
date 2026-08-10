"""Startup-order and default-resolution bugs, which only bite on real hardware.

Two crashes on the first real-robot launch motivated this file, and neither could
have been caught by testing pure functions:

- ``DoosanArm.__init__`` called ``self._log()`` before assigning
  ``self.logger``, so it died with AttributeError the moment the TCP setup tried
  to report itself. Only reachable with a live Doosan driver.
- ``handeye_calibration: ""`` in system.yaml overrode the node's default, and
  "empty means use the packaged file" was written in a comment but never in the
  code. ``Path("")`` became ``"."`` and numpy tried to load a directory.

Neither node can be constructed here (one needs the Doosan driver, the other a
GPU and ROS), so these tests check the two things that were actually wrong:
attribute order within the constructor, and how an empty path parameter resolves.
"""

import ast
import unittest
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parent.parent / "vla_system"


def constructor_of(module_path: Path, class_name: str) -> ast.FunctionDef:
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == "__init__":
                    return item
    raise AssertionError(f"{class_name}.__init__ not found in {module_path}")


def first_line_of_self_attribute_assignment(function: ast.FunctionDef, name: str):
    for node in ast.walk(function):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if (
                    isinstance(target, ast.Attribute)
                    and target.attr == name
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"
                ):
                    return node.lineno
    return None


def first_line_calling_self_method(function: ast.FunctionDef, name: str):
    for node in ast.walk(function):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == name
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "self"
        ):
            return node.lineno
    return None


class LoggerOrderTest(unittest.TestCase):
    """`self.logger` must exist before anything logs through it."""

    def test_doosan_arm_assigns_logger_before_logging(self):
        constructor = constructor_of(SOURCE_ROOT / "robot/moves.py", "DoosanArm")
        assigned = first_line_of_self_attribute_assignment(constructor, "logger")
        logged = first_line_calling_self_method(constructor, "_log")
        self.assertIsNotNone(assigned, "DoosanArm.__init__ must assign self.logger")
        if logged is not None:
            self.assertLess(
                assigned,
                logged,
                "DoosanArm.__init__ logs before self.logger is assigned; that "
                "raises AttributeError on every real-robot startup",
            )

    def test_dry_run_arm_assigns_logger_before_logging(self):
        constructor = constructor_of(SOURCE_ROOT / "robot/moves.py", "DryRunArm")
        assigned = first_line_of_self_attribute_assignment(constructor, "logger")
        logged = first_line_calling_self_method(constructor, "_log")
        self.assertIsNotNone(assigned)
        if logged is not None:
            self.assertLess(assigned, logged)

    def test_profile_is_assigned_before_it_is_used_for_logging(self):
        """The TCP setup reads profile.tool_name; profile must be set by then."""
        constructor = constructor_of(SOURCE_ROOT / "robot/moves.py", "DoosanArm")
        assigned = first_line_of_self_attribute_assignment(constructor, "profile")
        self.assertIsNotNone(assigned)

    def test_doosan_arm_binds_both_pose_factories(self):
        """Production must not rely on tests manually injecting ``posj``."""
        constructor = constructor_of(SOURCE_ROOT / "robot/moves.py", "DoosanArm")
        assignments = {}
        for node in ast.walk(constructor):
            if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.Name):
                continue
            for target in node.targets:
                if (
                    isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"
                    and target.attr in {"posj", "posx"}
                ):
                    assignments[target.attr] = node.value.id
        self.assertEqual(assignments, {"posj": "posj", "posx": "posx"})


class DoosanThreadingContractTest(unittest.TestCase):
    """Only the motion worker may touch the Doosan API.

    `moves.py` states the rule: Doosan's Python API funnels every call through
    `rclpy.spin_until_future_complete`, so two threads driving it raise
    "generator already executing". A 20 Hz timer that read the arm's pose was
    exactly that second thread, and it killed the first real grasp with
    `ValueError: generator already executing`.

    A timer callback runs on the node's executor, never on the worker, so no
    timer callback may reach `self.arm`.
    """

    def setUp(self):
        self.source = (SOURCE_ROOT / "nodes/robot_node.py").read_text(encoding="utf-8")
        self.tree = ast.parse(self.source)
        self.constructor = constructor_of(SOURCE_ROOT / "nodes/robot_node.py", "RobotNode")

    def timer_callback_names(self):
        names = []
        for node in ast.walk(self.constructor):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "create_timer"
            ):
                for argument in node.args[1:]:
                    if isinstance(argument, ast.Attribute):
                        names.append(argument.attr)
        return names

    def method_named(self, name):
        for node in ast.walk(self.tree):
            if isinstance(node, ast.FunctionDef) and node.name == name:
                return node
        return None

    def references_self_arm(self, function) -> bool:
        for node in ast.walk(function):
            if (
                isinstance(node, ast.Attribute)
                and node.attr == "arm"
                and isinstance(node.value, ast.Name)
                and node.value.id == "self"
            ):
                return True
        return False

    def test_no_timer_callback_touches_the_arm(self):
        offenders = []
        for name in self.timer_callback_names():
            function = self.method_named(name)
            if function is not None and self.references_self_arm(function):
                offenders.append(name)
        self.assertEqual(
            offenders,
            [],
            f"timer callbacks {offenders} reach self.arm; the Doosan API may only "
            "be called from the motion worker, or it raises 'generator already "
            "executing' the moment a motion is in progress",
        )

    def test_the_initial_tcp_pose_rides_inside_the_grasp_request(self):
        """The first pose comes from the worker before continuous samples."""
        self.assertIn("request.tcp_posx = tcp_posx", self.source)

    def test_the_arm_is_read_where_the_tracking_request_is_built(self):
        function = self.method_named("begin_wrist_grasp")
        self.assertIsNotNone(function)
        self.assertTrue(
            self.references_self_arm(function),
            "begin_wrist_grasp must read its initial pose on the motion worker",
        )

    def test_the_timer_uses_an_independent_async_pose_client(self):
        function = self.method_named("publish_tcp_pose_sample")
        self.assertIsNotNone(function)
        self.assertFalse(self.references_self_arm(function))
        self.assertIn("self.tcp_pose_client.call_async", ast.unparse(function))

    def test_tcp_samples_are_correlated_to_the_active_request(self):
        function = self.method_named("_tcp_pose_response")
        self.assertIsNotNone(function)
        body = ast.unparse(function)
        self.assertIn("request_id != self.grasp_request_id", body)

    def test_a_hung_pose_future_is_discarded_and_retried(self):
        function = self.method_named("publish_tcp_pose_sample")
        body = ast.unparse(function)
        self.assertIn("tcp_pose_service_timeout_s", body)
        self.assertIn("stale.cancel()", body)

    def test_pose_stamp_uses_the_service_round_trip_midpoint(self):
        function = self.method_named("_tcp_pose_response")
        body = ast.unparse(function)
        self.assertIn("sent_ros_ns", body)
        self.assertIn("received_ros_ns", body)
        self.assertIn("// 2", body)


class ContinuousWristContractTest(unittest.TestCase):
    def setUp(self):
        self.robot_source = (
            SOURCE_ROOT / "nodes/robot_node.py"
        ).read_text(encoding="utf-8")
        self.wrist_source = (
            SOURCE_ROOT / "nodes/wrist_grasp_node.py"
        ).read_text(encoding="utf-8")

    @staticmethod
    def method(source, name):
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.FunctionDef) and node.name == name:
                return node
        raise AssertionError(f"method {name} not found")

    def test_target_is_registered_before_the_observation_move(self):
        body = ast.unparse(self.method(self.robot_source, "wrist_guided_pick"))
        self.assertLess(body.index("self.begin_wrist_grasp"), body.index("self.arm.observe"))

    def test_graspgenx_is_submitted_off_the_ros_callback_thread(self):
        tracking = ast.unparse(self.method(self.wrist_source, "_try_track"))
        compute = ast.unparse(self.method(self.wrist_source, "_compute_grasp"))
        self.assertIn("self.grasp_executor.submit", tracking)
        self.assertNotIn("self.grasp_client.generate", tracking)
        self.assertIn("self.grasp_client.generate", compute)

    def test_worker_result_is_generation_gated_before_publication(self):
        body = ast.unparse(self.method(self.wrist_source, "_poll_grasp_future"))
        self.assertIn("result_matches_session", body)
        self.assertLess(body.index("if not active"), body.index("self.plan_publisher.publish"))


class EmptyPathParameterTest(unittest.TestCase):
    """An empty path parameter must fall back, not become the current directory."""

    def test_empty_string_does_not_resolve_to_a_directory(self):
        self.assertEqual(str(Path("")), ".")
        self.assertTrue(Path(".").is_dir())  # which is why np.load failed

    def test_the_wrist_node_falls_back_on_an_empty_calibration_path(self):
        source = (SOURCE_ROOT / "nodes/wrist_grasp_node.py").read_text(encoding="utf-8")
        # The resolution must go through a truthiness check against the default,
        # not straight into the loader.
        self.assertIn("or default_calibration", source)
        self.assertNotIn(
            'load_camera_to_gripper(\n            str(self.get_parameter("handeye_calibration").value)\n        )',
            source,
            "the raw parameter is being loaded without an empty-string fallback",
        )

    def test_every_empty_string_path_parameter_in_config_has_a_fallback(self):
        """Config uses "" to mean "use the built-in default" in several places.

        Each one needs code that honours it. This catches the next parameter
        added with the same convention and no implementation.
        """
        import yaml

        config = yaml.safe_load(
            (SOURCE_ROOT.parent / "config/system.yaml").read_text(encoding="utf-8")
        )
        empty_path_params = []
        for node_name, block in config.items():
            for key, value in (block.get("ros__parameters") or {}).items():
                if value == "" and ("file" in key or "path" in key or "repo" in key or "calibration" in key):
                    empty_path_params.append((node_name, key))
        # Documented, so a reviewer can see which ones rely on a code fallback.
        self.assertEqual(
            sorted(empty_path_params),
            [
                ("vla_agent", "env_file"),
                ("vla_robot", "handeye_calibration"),
                ("vla_wrist", "graspgen_repo"),
                ("vla_wrist", "handeye_calibration"),
            ],
        )


if __name__ == "__main__":
    unittest.main()

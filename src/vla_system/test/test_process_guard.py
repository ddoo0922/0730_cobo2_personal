"""Leftover-pipeline cleanup: what gets signalled, how hard, and what is spared.

The GUI runs this at every start, so a bug here either leaves a second
`vla_pick_bridge_node` racing cobot2_ws's pick_fsm or kills something it had
no business touching.
"""

import os
import re
import signal
import subprocess
import sys
import time
import unittest

from vla_system.process_guard import (
    EXISTING_PIPELINE_PATTERN,
    PIPELINE_PATTERN,
    REALSENSE_PATTERN,
    ancestor_pids,
    escalate_termination,
    parent_pid,
    parse_pgrep_output,
    process_is_alive,
)


class FakeProcessTable:
    """Processes that die at a chosen point in the escalation."""

    def __init__(self, dies_on):
        # dies_on: {pid: signal number it obeys, or None for unkillable}
        self.dies_on = dict(dies_on)
        self.alive = set(dies_on)
        self.received = []
        self.waits = []

    def send_signal(self, pid, signal_number):
        self.received.append((pid, signal_number))
        if self.dies_on.get(pid) == signal_number:
            self.alive.discard(pid)

    def is_alive(self, pid):
        return pid in self.alive

    def wait(self, seconds):
        self.waits.append(seconds)


class EscalationTest(unittest.TestCase):
    def test_a_well_behaved_node_dies_on_sigint_and_is_not_hit_again(self):
        table = FakeProcessTable({101: signal.SIGINT, 102: signal.SIGINT})
        survivors = escalate_termination(
            [101, 102],
            send_signal=table.send_signal,
            is_alive=table.is_alive,
            wait=table.wait,
        )
        self.assertEqual(survivors, [])
        self.assertEqual({s for _pid, s in table.received}, {signal.SIGINT})

    def test_nothing_is_waited_on_when_sigint_is_enough(self):
        """The common case must not spend the whole 4 second budget."""
        table = FakeProcessTable({101: signal.SIGINT})
        escalate_termination(
            [101],
            send_signal=table.send_signal,
            is_alive=table.is_alive,
            wait=table.wait,
        )
        self.assertEqual(table.waits, [])

    def test_a_wedged_node_is_escalated_to_sigkill(self):
        table = FakeProcessTable({101: signal.SIGKILL})
        survivors = escalate_termination(
            [101],
            send_signal=table.send_signal,
            is_alive=table.is_alive,
            wait=table.wait,
            escalation=((signal.SIGINT, 0.2), (signal.SIGTERM, 0.2), (signal.SIGKILL, 0.2)),
            poll_interval_s=0.1,
        )
        self.assertEqual(survivors, [])
        self.assertEqual(
            [s for _pid, s in table.received],
            [signal.SIGINT, signal.SIGTERM, signal.SIGKILL],
        )

    def test_an_unkillable_process_is_reported_not_hidden(self):
        """The GUI blocks the start on this: a live robot_node must not be
        joined by a second one."""
        table = FakeProcessTable({101: signal.SIGINT, 102: None})
        survivors = escalate_termination(
            [101, 102],
            send_signal=table.send_signal,
            is_alive=table.is_alive,
            wait=table.wait,
            escalation=((signal.SIGINT, 0.1), (signal.SIGKILL, 0.1)),
            poll_interval_s=0.1,
        )
        self.assertEqual(survivors, [102])
        # 101 obeyed SIGINT, so it must not appear in the SIGKILL round.
        self.assertNotIn((101, signal.SIGKILL), table.received)

    def test_an_already_dead_pid_is_never_signalled(self):
        """Reusing a stale pid list must not fire signals at whatever now owns
        that number."""
        table = FakeProcessTable({})
        survivors = escalate_termination(
            [999],
            send_signal=table.send_signal,
            is_alive=table.is_alive,
            wait=table.wait,
        )
        self.assertEqual(survivors, [])
        self.assertEqual(table.received, [])

    def test_an_empty_list_is_a_no_op(self):
        table = FakeProcessTable({})
        self.assertEqual(
            escalate_termination(
                [],
                send_signal=table.send_signal,
                is_alive=table.is_alive,
                wait=table.wait,
            ),
            [],
        )
        self.assertEqual(table.received, [])


class LivenessTest(unittest.TestCase):
    """Uses real processes, but only `sleep` -- no ROS, camera or robot."""

    def test_a_running_process_is_alive(self):
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
        try:
            self.assertTrue(process_is_alive(process.pid))
        finally:
            process.kill()
            process.wait(timeout=5)

    def test_an_unreaped_child_is_not_alive(self):
        """A zombie has exited. Counting it as alive made the GUI report its own
        just-killed pipeline as unkillable and refuse to restart."""
        process = subprocess.Popen([sys.executable, "-c", ""])
        try:
            deadline = time.monotonic() + 5.0
            while process_is_alive(process.pid) and time.monotonic() < deadline:
                time.sleep(0.02)
            # Deliberately not waited yet: the child is a zombie right now.
            self.assertFalse(process_is_alive(process.pid))
        finally:
            process.wait(timeout=5)

    def test_a_pid_that_does_not_exist_is_not_alive(self):
        pid = 0x7FFFFFFF
        if os.path.exists(f"/proc/{pid}"):
            self.skipTest("pid unexpectedly in use")
        self.assertFalse(process_is_alive(pid))


class AncestryTest(unittest.TestCase):
    """Never signal yourself or anything you are running inside of."""

    def test_the_chain_includes_this_process_and_its_parent(self):
        ancestors = ancestor_pids()
        self.assertIn(os.getpid(), ancestors)
        self.assertIn(os.getppid(), ancestors)

    def test_the_chain_reaches_init(self):
        self.assertIn(1, ancestor_pids())

    def test_a_child_sees_this_process_as_an_ancestor(self):
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
        try:
            self.assertEqual(parent_pid(process.pid), os.getpid())
            self.assertIn(os.getpid(), ancestor_pids(process.pid))
        finally:
            process.kill()
            process.wait(timeout=5)

    def test_an_ancestor_is_filtered_out_of_pgrep_output(self):
        """The bug this guards: a wrapper shell started as
        `bash -c "... ros2 launch vla_system ..."` carries the launch command in
        its own command line, so pgrep lists it as if it were the pipeline."""
        parent = os.getppid()
        output = (
            f"{parent} bash -c ros2 launch vla_system vla_system.launch.py\n"
            "31337 ros2 launch vla_system vla_system.launch.py\n"
        )
        self.assertEqual(
            parse_pgrep_output(output, exclude=ancestor_pids()),
            [(31337, "ros2 launch vla_system vla_system.launch.py")],
        )

    def test_a_missing_pid_has_no_parent(self):
        pid = 0x7FFFFFFF
        if os.path.exists(f"/proc/{pid}"):
            self.skipTest("pid unexpectedly in use")
        self.assertIsNone(parent_pid(pid))


class PgrepParsingTest(unittest.TestCase):
    def test_pid_and_full_command_are_split(self):
        output = (
            "8396 /usr/bin/python3 /opt/vla_system/lib/vla_system/perception_node --ros-args\n"
            "8400 ros2 launch vla_system vla_system.launch.py\n"
        )
        self.assertEqual(
            parse_pgrep_output(output),
            [
                (8396, "/usr/bin/python3 /opt/vla_system/lib/vla_system/perception_node --ros-args"),
                (8400, "ros2 launch vla_system vla_system.launch.py"),
            ],
        )

    def test_the_caller_can_exclude_itself(self):
        """A shell whose command line contains the pattern does get listed, and
        signalling our own process group would kill the GUI mid-cleanup."""
        output = "111 bash -c pgrep -af vla_system/lib/vla_system/robot_node\n222 robot_node\n"
        self.assertEqual(parse_pgrep_output(output, exclude={111}), [(222, "robot_node")])

    def test_blank_and_malformed_lines_are_dropped(self):
        output = "\n   \nnotapid something\n333 robot_node\n444\n"
        self.assertEqual(parse_pgrep_output(output), [(333, "robot_node")])


class PatternTest(unittest.TestCase):
    """The pattern decides what dies. It must not include the GUI itself."""

    def matches(self, command):
        return re.search(EXISTING_PIPELINE_PATTERN, command) is not None

    def test_the_pipeline_nodes_and_launch_are_matched(self):
        for command in (
            "ros2 launch vla_system vla_system.launch.py enable_pick_bridge:=true",
            "/usr/bin/python3 /x/install/vla_system/lib/vla_system/perception_node --ros-args",
            "/usr/bin/python3 /x/install/vla_system/lib/vla_system/agent_node",
            # Two of these would both publish/subscribe /vla/robot/action and
            # /vla/pick_command, each handing cobot2_ws a different idea of
            # what to pick (2026-08-11 -- this line was missing until a
            # real-hardware session showed a manually-started
            # vla_pick_bridge_node survived the GUI's own "leftover" cleanup).
            "/usr/bin/python3 /x/install/vla_system/lib/vla_system/vla_pick_bridge_node",
            # Our launch file starts this one, and there is only one camera.
            "/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node --ros-args",
        ):
            with self.subTest(command=command):
                self.assertTrue(self.matches(command))

    def test_a_stale_realsense_node_is_matched(self):
        """It survived cleanup once and the next start died on "Device or
        resource busy": the new node's claim resets the USB device out from under
        the old one."""
        self.assertTrue(
            self.matches(
                "/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node "
                "--ros-args -r __node:=camera -r __ns:=/camera"
            )
        )

    def test_the_gui_is_spared(self):
        self.assertFalse(
            self.matches("/usr/bin/python3 /x/install/vla_system/lib/vla_system/vla_gui")
        )

    def test_an_unrelated_ros_process_is_spared(self):
        for command in (
            "ros2 launch m0609_rg2_bringup bringup.launch.py",
            "/home/rokey/cobot_ws/install/dsr_bringup2/lib/dsr_bringup2/dsr_bringup2",
        ):
            with self.subTest(command=command):
                self.assertFalse(self.matches(command))


class SplitPatternTest(unittest.TestCase):
    """PIPELINE_PATTERN vs REALSENSE_PATTERN, split so the GUI can leave a
    camera it does not own alone in cobot2_ws-integration mode (2026-08-11 --
    a real-hardware session had the GUI kill a manually-started `reals1280`
    camera and a manually-started pipeline it had no business touching, both
    because `include_realsense` did not exist yet)."""

    def test_realsense_pattern_matches_only_the_camera_node(self):
        self.assertIsNotNone(
            re.search(
                REALSENSE_PATTERN,
                "/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node",
            )
        )
        self.assertIsNone(re.search(REALSENSE_PATTERN, "ros2 launch vla_system vla_system.launch.py"))

    def test_pipeline_pattern_alone_spares_the_camera(self):
        """This is what `include_realsense=False` searches with -- a camera
        the GUI did not start (someone's own launch/alias) must not appear."""
        self.assertIsNone(
            re.search(
                PIPELINE_PATTERN,
                "/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node",
            )
        )

    def test_pipeline_pattern_still_catches_the_launch_and_nodes(self):
        for command in (
            "ros2 launch vla_system vla_system.launch.py enable_pick_bridge:=true",
            "/x/install/vla_system/lib/vla_system/perception_node",
            "/x/install/vla_system/lib/vla_system/agent_node",
            "/x/install/vla_system/lib/vla_system/vla_pick_bridge_node",
        ):
            with self.subTest(command=command):
                self.assertIsNotNone(re.search(PIPELINE_PATTERN, command))

    def test_the_two_patterns_together_equal_the_combined_default(self):
        """EXISTING_PIPELINE_PATTERN (include_realsense=True's search string)
        must not silently drift from the two halves it is built from."""
        self.assertEqual(
            EXISTING_PIPELINE_PATTERN, f"{PIPELINE_PATTERN}|{REALSENSE_PATTERN}"
        )


if __name__ == "__main__":
    unittest.main()

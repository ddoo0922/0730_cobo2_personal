"""Finding and killing leftover pipeline processes.

Split out of ``vla_gui`` so it can be tested without tkinter, OpenCV or a ROS
runtime: the escalation is the part that has to be right, and it is pure logic
once the signal and liveness calls are injected.

Why it exists at all: closing or killing the GUI does not stop the ``ros2
launch`` subprocess it started -- there is no OS-level parent-death link back to
the GUI -- so a fresh GUI has no memory of the previous run and would happily
launch a second full pipeline. Two ``vla_pick_bridge_node``s means two
executors sending cobot2_ws different ideas of what to pick.
"""

import os
import signal
import subprocess
import time


# Matches this pipeline's node executables and the launch process itself. It
# deliberately does not match `vla_gui`, which must survive its own cleanup.
#
# robot_node/wrist_grasp_node were removed (CLAUDE.md #3, cobot2_ws's pick_fsm
# is the sole executor now) so they are gone from this pattern too --
# vla_pick_bridge_node is the only thing left that can race another instance
# of itself over /vla/robot/action and /vla/pick_command.
PIPELINE_PATTERN = (
    r"ros2 launch vla_system vla_system\.launch\.py"
    r"|vla_system/lib/vla_system/(perception_node|agent_node|vla_pick_bridge_node)"
)

# Split out from PIPELINE_PATTERN (2026-08-11) so the GUI can leave someone
# else's RealSense process alone when it is not about to need the device
# itself (cobot2_ws-integration mode always launches with
# enable_realsense:=false -- see vla_gui.py's `_clear_leftover_pipeline`).
#
# realsense2_camera_node is not one of our nodes, but our launch file starts
# one and there is exactly one physical camera. Leaving a stale one alive does
# not merely waste a process: the new node's attempt to claim the device
# resets the USB connection, the old node reports "device has been
# disconnected", and the new one dies on "Device or resource busy". Two
# RealSense nodes can never usefully coexist -- but only when *we* are the one
# about to open it.
REALSENSE_PATTERN = r"realsense2_camera/realsense2_camera_node"

EXISTING_PIPELINE_PATTERN = f"{PIPELINE_PATTERN}|{REALSENSE_PATTERN}"

# Every start clears leftovers, so the escalation has to finish quickly enough
# that the window does not look hung. SIGINT is where rclpy shuts down cleanly
# and releases the gripper client; the harder signals exist for a node already
# wedged in a C call that never returns.
TERMINATION_ESCALATION = (
    (signal.SIGINT, 4.0),
    (signal.SIGTERM, 2.0),
    (signal.SIGKILL, 1.0),
)
POLL_INTERVAL_S = 0.1


def parent_pid(pid: int):
    """The ppid from /proc, or None if it cannot be read."""

    try:
        with open(f"/proc/{pid}/stat", "rb") as handle:
            # Skip the parenthesised comm field: state, then ppid.
            fields = handle.read().rsplit(b")", 1)[-1].split()
        return int(fields[1])
    except (OSError, IndexError, ValueError):
        return None


def ancestor_pids(pid=None) -> set:
    """This process and every process above it, up to init.

    Signalling an ancestor is never right, and it is easy to hit by accident: a
    wrapper shell started as ``bash -c "... ros2 launch vla_system ..."`` carries
    the launch command in its own command line, so pgrep lists it alongside the
    real pipeline. Killing it takes down the caller mid-cleanup.
    """

    current = os.getpid() if pid is None else pid
    seen = set()
    while current and current not in seen:
        seen.add(current)
        current = parent_pid(current)
    return seen


def find_existing_pipeline_pids(pattern: str = EXISTING_PIPELINE_PATTERN):
    """Return (pid, command line) per already-running pipeline process."""

    try:
        result = subprocess.run(
            ["pgrep", "-af", pattern],
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    if result.returncode != 0:
        return []
    return parse_pgrep_output(result.stdout, exclude=ancestor_pids())


def parse_pgrep_output(output: str, exclude=frozenset()):
    """Parse ``pgrep -af`` lines into (pid, command line) pairs.

    ``exclude`` keeps the caller from signalling itself or anything it depends
    on: pgrep never lists its own process, but a shell whose command line
    happens to contain the pattern does get listed. See ``ancestor_pids``.
    """

    found = []
    for line in output.splitlines():
        pid_text, _, command = line.strip().partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid in exclude or not command:
            continue
        found.append((pid, command))
    return found


def process_is_alive(pid: int) -> bool:
    """True while the pid can still run.

    A zombie does not count as alive: it has already exited and is only waiting
    for its parent to collect the status. This matters because ``os.kill(pid,
    0)`` *succeeds* on a zombie, so a signal probe alone can never confirm the
    death of a process we started ourselves -- the GUI would report its own
    just-killed pipeline as unkillable and refuse to restart. Leftovers from a
    previous GUI are reparented to init and reaped immediately, which is why the
    bug only shows on the processes we still own.
    """

    try:
        with open(f"/proc/{pid}/stat", "rb") as handle:
            # The comm field is parenthesised and may itself contain spaces and
            # parentheses, so split after its closing paren; state comes next.
            fields = handle.read().rsplit(b")", 1)[-1].split()
        return bool(fields) and fields[0] != b"Z"
    except FileNotFoundError:
        return False
    except OSError:
        pass  # /proc unreadable; fall back to the signal probe below.

    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        # Someone else's process: it exists, we just cannot touch it.
        return True
    return True


def send_signal_quietly(pid: int, signal_number: int) -> None:
    try:
        os.kill(pid, signal_number)
    except (ProcessLookupError, PermissionError):
        pass


def escalate_termination(
    pids,
    send_signal=send_signal_quietly,
    is_alive=process_is_alive,
    wait=time.sleep,
    escalation=TERMINATION_ESCALATION,
    poll_interval_s: float = POLL_INTERVAL_S,
):
    """Signal each pid with progressively harder signals until it is gone.

    Stops as soon as everything has exited, so the common case -- nodes that
    honour SIGINT -- costs a fraction of a second rather than the full budget.

    Returns the pids still alive after the last step. A non-empty result means
    something is unkillable (another user's process, or stuck in uninterruptible
    IO) and the caller must not start a second pipeline on top of it.
    """

    remaining = [pid for pid in pids if is_alive(pid)]
    for signal_number, timeout_s in escalation:
        if not remaining:
            break
        for pid in remaining:
            send_signal(pid, signal_number)
        waited = 0.0
        while True:
            remaining = [pid for pid in remaining if is_alive(pid)]
            if not remaining or waited >= timeout_s:
                break
            wait(poll_interval_s)
            waited += poll_interval_s
    return remaining

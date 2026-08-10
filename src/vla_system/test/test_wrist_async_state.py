"""Cancellation and exactly-once gates around asynchronous GraspGenX work."""

from collections import deque
from concurrent.futures import Future
from time import monotonic, perf_counter
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from vla_system.nodes.wrist_grasp_node import GraspComputation, WristGraspNode


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Clock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds)


def _success():
    return GraspComputation(
        success=True,
        target_posx=(400.0, 20.0, 80.0, 0.0, 180.0, 0.0),
        pregrasp_posx=(400.0, 20.0, 120.0, 0.0, 180.0, 0.0),
        confidence=0.9,
        approach_tilt_deg=3.0,
        candidate_count=7,
        cloud_points=1234,
    )


def _harness(*, active=True, matching_generation=True):
    request = SimpleNamespace(request_id="apple:1", object_id="apple_1")
    future = Future()
    source_stamp_ns = 1_000_000_000
    failures = []
    node = SimpleNamespace(
        grasp_future=future,
        planning_request=request,
        planning_generation=4,
        planning_started=perf_counter() - 0.1,
        planning_source_stamp_ns=source_stamp_ns,
        tracking_busy=True,
        pending_request=request if active else None,
        session_generation=4 if matching_generation else 5,
        tracking_deadline=monotonic() + 10.0,
        plan_publisher=_Publisher(),
        durations=deque(maxlen=5),
        get_clock=lambda: _Clock(source_stamp_ns + 2_000_000_000),
        get_logger=lambda: Mock(),
    )

    def clear_tracking():
        node.pending_request = None

    def finish_failure(failed_request, reason):
        failures.append((failed_request.request_id, reason))

    node._clear_tracking = clear_tracking
    node._finish_failure = finish_failure
    return node, future, failures


class AsyncGraspResultTest(unittest.TestCase):
    def test_active_result_is_published_once_with_the_source_stamp(self):
        node, future, failures = _harness()
        future.set_result(_success())

        WristGraspNode._poll_grasp_future(node)
        WristGraspNode._poll_grasp_future(node)

        self.assertEqual(failures, [])
        self.assertEqual(len(node.plan_publisher.messages), 1)
        plan = node.plan_publisher.messages[0]
        self.assertEqual(plan.request_id, "apple:1")
        self.assertEqual(plan.header.stamp.sec, 1)
        self.assertEqual(plan.header.stamp.nanosec, 0)
        self.assertEqual(plan.header.frame_id, "base")
        self.assertIsNone(node.grasp_future)
        self.assertFalse(node.tracking_busy)

    def test_cancelled_result_is_discarded(self):
        node, future, failures = _harness(active=False)
        future.set_result(_success())

        WristGraspNode._poll_grasp_future(node)

        self.assertEqual(node.plan_publisher.messages, [])
        self.assertEqual(failures, [])

    def test_previous_generation_cannot_publish_into_a_new_session(self):
        node, future, failures = _harness(matching_generation=False)
        future.set_result(_success())

        WristGraspNode._poll_grasp_future(node)

        self.assertEqual(node.plan_publisher.messages, [])
        self.assertEqual(failures, [])

    def test_worker_exception_becomes_one_correlated_failure(self):
        node, future, failures = _harness()
        future.set_exception(RuntimeError("boom"))

        WristGraspNode._poll_grasp_future(node)

        self.assertEqual(node.plan_publisher.messages, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("boom", failures[0][1])

    def test_result_after_deadline_is_never_published(self):
        node, future, failures = _harness()
        node.tracking_deadline = monotonic() - 0.01
        future.set_result(_success())

        WristGraspNode._poll_grasp_future(node)

        self.assertEqual(node.plan_publisher.messages, [])
        self.assertEqual(len(failures), 1)
        self.assertIn("deadline", failures[0][1])


if __name__ == "__main__":
    unittest.main()

"""Conversation memory is the state machine now, so it gets tested like one."""

import json
import unittest

from vla_system.agent.conversation import (
    Conversation,
    build_situation,
    robot_state_to_payload,
    scene_to_payload,
)


class FakePoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class FakeObject:
    def __init__(self, object_id, class_name, valid=True, color="red", xyz=(0.4, -0.1, 0.05)):
        self.id = object_id
        self.class_name = class_name
        self.position_valid = valid
        self.color = color
        self.position_base = FakePoint(*xyz)


class FakeScene:
    def __init__(self, objects, calibration_ok=True):
        self.objects = objects
        self.calibration_ok = calibration_ok


class FakeState:
    def __init__(self, **kwargs):
        self.status = kwargs.get("status", "idle")
        self.holding_object_id = kwargs.get("holding_object_id", "")
        self.holding_class_name = kwargs.get("holding_class_name", "")
        self.current_action = kwargs.get("current_action", "")
        self.current_action_id = kwargs.get("current_action_id", "")
        self.last_action = kwargs.get("last_action", "")
        self.last_result = kwargs.get("last_result", "")
        self.details = kwargs.get("details", "")
        self.motion_enabled = kwargs.get("motion_enabled", False)


class ScenePayloadTest(unittest.TestCase):
    def test_positions_are_rounded_to_millimetres(self):
        scene = FakeScene([FakeObject("apple_17", "apple", xyz=(0.412345, -0.18, 0.05))])
        payload = scene_to_payload(scene)
        self.assertEqual(payload["visible_objects"][0]["position_base"], [0.412, -0.18, 0.05])

    def test_unpositioned_objects_are_still_pickable(self):
        """cobot2_ws's pick_fsm computes its own grasp coordinate from the
        class name alone (bridge/pick_bridge.py) -- this ws's own
        position_valid/calibration has nothing to do with whether an object
        can be picked, so every visible object is reported pickable."""
        scene = FakeScene([FakeObject("apple_17", "apple", valid=False)])
        entry = scene_to_payload(scene)["visible_objects"][0]
        self.assertTrue(entry["pickable"])
        self.assertNotIn("position_base", entry)

    def test_missing_scene_says_so_instead_of_looking_empty(self):
        payload = scene_to_payload(None)
        self.assertEqual(payload["visible_objects"], [])
        self.assertIn("note", payload)


class RobotStatePayloadTest(unittest.TestCase):
    def test_empty_gripper_reports_holding_none(self):
        self.assertIsNone(robot_state_to_payload(FakeState())["holding"])

    def test_held_object_is_reported_with_its_class(self):
        payload = robot_state_to_payload(
            FakeState(holding_object_id="apple_17", holding_class_name="apple")
        )
        self.assertEqual(payload["holding"], {"id": "apple_17", "class": "apple"})

    def test_unknown_state_is_not_silently_idle(self):
        self.assertEqual(robot_state_to_payload(None)["status"], "unknown")


class ConversationTest(unittest.TestCase):
    def test_situation_is_valid_json_with_all_three_sections(self):
        payload = json.loads(
            build_situation({"type": "user_said"}, {"visible_objects": []}, {"status": "idle"})
        )
        self.assertEqual(set(payload), {"event", "scene", "robot_state"})

    def test_empty_assistant_text_is_not_stored(self):
        conversation = Conversation()
        conversation.add_assistant("   ")
        self.assertEqual(len(conversation), 0)

    def test_trimming_never_orphans_a_tool_output(self):
        """A function_call_output whose call fell off the front is a 400."""
        conversation = Conversation(max_items=6)
        for turn in range(6):
            conversation.add_user(f"turn {turn}")
            conversation.add_function_call(f"call{turn}", "wait", "{}")
            conversation.add_function_output(f"call{turn}", "ok")

        seen_calls = set()
        for item in conversation.items():
            if item.get("type") == "function_call":
                seen_calls.add(item["call_id"])
            if item.get("type") == "function_call_output":
                self.assertIn(item["call_id"], seen_calls)

    def test_trimming_cuts_at_a_user_message(self):
        conversation = Conversation(max_items=4)
        for turn in range(5):
            conversation.add_user(f"turn {turn}")
            conversation.add_assistant("ok")
        self.assertEqual(conversation.items()[0].get("role"), "user")
        self.assertLessEqual(len(conversation), 4)

    def test_history_stays_bounded(self):
        conversation = Conversation(max_items=10)
        for turn in range(50):
            conversation.add_user(f"turn {turn}")
            conversation.add_assistant("ok")
        self.assertLessEqual(len(conversation), 10)


if __name__ == "__main__":
    unittest.main()

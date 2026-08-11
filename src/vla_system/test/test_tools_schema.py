"""The tool schemas are the API contract with the model; a typo breaks a turn."""

import unittest

from vla_system.agent.tools import MOTION_TOOLS, TERMINAL_TOOLS, TOOL_NAMES, TOOLS


class ToolSchemaTest(unittest.TestCase):
    def test_every_tool_the_plan_calls_for_exists(self):
        expected = {
            "pick_and_place",
            "cancel_current_action",
            "ask_clarification",
            "wait",
        }
        self.assertEqual(set(TOOL_NAMES), expected)

    def test_names_are_unique(self):
        self.assertEqual(len(TOOL_NAMES), len(set(TOOL_NAMES)))

    def test_schemas_are_in_the_flat_responses_api_shape(self):
        for tool in TOOLS:
            self.assertEqual(tool["type"], "function")
            self.assertIn("name", tool)
            self.assertIn("parameters", tool)

    def test_strict_mode_requires_every_property(self):
        """With strict=True the API rejects an optional property outright."""
        for tool in TOOLS:
            if not tool.get("strict"):
                continue
            parameters = tool["parameters"]
            self.assertFalse(parameters["additionalProperties"], tool["name"])
            self.assertEqual(
                set(parameters["properties"]), set(parameters["required"]), tool["name"]
            )

    def test_every_tool_is_described_for_the_model(self):
        for tool in TOOLS:
            self.assertTrue(tool.get("description", "").strip(), tool["name"])

    def test_motion_and_terminal_tools_are_real_tools(self):
        for name in MOTION_TOOLS + TERMINAL_TOOLS:
            self.assertIn(name, TOOL_NAMES)

    def test_picking_tools_take_an_object_handle(self):
        for tool in TOOLS:
            if tool["name"] == "pick_and_place":
                self.assertIn("object_id", tool["parameters"]["properties"])

    def test_every_tool_carries_a_sentence_for_the_user(self):
        """Without this the robot can act, or stop, in total silence."""
        for tool in TOOLS:
            properties = tool["parameters"]["properties"]
            self.assertTrue(
                {"say", "question"} & set(properties),
                f"{tool['name']} has nothing to tell the user",
            )

    def test_pick_and_place_carries_a_destination(self):
        tool = next(t for t in TOOLS if t["name"] == "pick_and_place")
        properties = tool["parameters"]["properties"]
        self.assertEqual(
            set(properties["place"]["enum"]), {"basket", "table", "discard"}
        )

    def test_clarification_can_carry_candidates_for_the_gui_to_crop(self):
        tool = next(t for t in TOOLS if t["name"] == "ask_clarification")
        properties = tool["parameters"]["properties"]
        self.assertEqual(properties["object_ids"]["type"], "array")
        self.assertIn("question", properties)


if __name__ == "__main__":
    unittest.main()

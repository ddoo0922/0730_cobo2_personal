"""The JSON boundary to cobot2_ws, tested without a live cobot2_ws process."""

import unittest

from vla_system.bridge.pick_bridge import (
    FSM_HOLDING_STATES,
    PLACE_VALUES,
    UNVERIFIED_PLACE_VALUES,
    bbox_center,
    build_abort_command,
    build_pick_command,
    find_class_name,
    find_scene_object,
    fsm_state_view,
    parse_pick_result,
    place_rejection_reason,
    result_update,
)


class FakeSceneObject:
    def __init__(self, object_id, class_name, bbox=(0.0, 0.0, 0.0, 0.0)):
        self.id = object_id
        self.class_name = class_name
        self.x_min, self.y_min, self.x_max, self.y_max = bbox


class BuildPickCommandTest(unittest.TestCase):
    def test_shape_matches_the_contract(self):
        payload = build_pick_command(
            class_name="apple", place="basket", request_id="a17-3", stamp_ns=123
        )
        self.assertEqual(
            payload,
            {
                "cmd": "pick_and_place",
                "class": "apple",
                "place": "basket",
                "request_id": "a17-3",
                "stamp_ns": 123,
            },
        )

    def test_place_is_always_included_now(self):
        payload = build_pick_command(
            class_name="apple", place="table", request_id="x", stamp_ns=0
        )
        self.assertEqual(payload["place"], "table")

    def test_pixel_omitted_when_not_given(self):
        """Matches contract #2: an absent pixel is a valid, common case (no
        frame resolution yet), not an error -- must not send a partial/None
        pixel field cobot2_ws would have to special-case."""
        payload = build_pick_command(
            class_name="apple", place="basket", request_id="x", stamp_ns=0
        )
        self.assertNotIn("pixel", payload)
        self.assertNotIn("pixel_wh", payload)

    def test_pixel_included_when_given(self):
        payload = build_pick_command(
            class_name="apple",
            place="basket",
            request_id="x",
            stamp_ns=0,
            pixel=(320.0, 240.0),
            pixel_wh=(640, 480),
        )
        self.assertEqual(payload["pixel"], [320.0, 240.0])
        self.assertEqual(payload["pixel_wh"], [640, 480])

    def test_pixel_and_pixel_wh_are_both_or_neither(self):
        """build_pick_command itself doesn't enforce this -- the caller
        (vla_pick_bridge_node) does, by only ever passing both or neither.
        This just pins the shape: passing one alone still gets dropped."""
        payload = build_pick_command(
            class_name="apple",
            place="basket",
            request_id="x",
            stamp_ns=0,
            pixel=(1.0, 2.0),
            pixel_wh=None,
        )
        self.assertNotIn("pixel", payload)


class PlaceRejectionReasonTest(unittest.TestCase):
    def test_basket_is_always_fine(self):
        self.assertIsNone(place_rejection_reason("basket", allow_unverified=False))
        self.assertIsNone(place_rejection_reason("basket", allow_unverified=True))

    def test_unverified_values_blocked_by_default(self):
        for place in UNVERIFIED_PLACE_VALUES:
            reason = place_rejection_reason(place, allow_unverified=False)
            self.assertIsNotNone(reason)
            self.assertIn(place, reason)

    def test_unverified_values_allowed_when_flagged(self):
        for place in UNVERIFIED_PLACE_VALUES:
            self.assertIsNone(place_rejection_reason(place, allow_unverified=True))

    def test_unknown_value_rejected_regardless_of_flag(self):
        reason = place_rejection_reason("shelf", allow_unverified=True)
        self.assertIsNotNone(reason)

    def test_contract_values_match(self):
        """PLACE_VALUES must mirror cobot2_ws's own copy (contract #5)."""
        self.assertEqual(PLACE_VALUES, frozenset({"basket", "table", "discard"}))
        self.assertTrue(UNVERIFIED_PLACE_VALUES <= PLACE_VALUES)


class BuildAbortCommandTest(unittest.TestCase):
    def test_shape(self):
        payload = build_abort_command(request_id="r1", reason="정지", stamp_ns=5)
        self.assertEqual(payload["cmd"], "abort")
        self.assertEqual(payload["request_id"], "r1")
        self.assertEqual(payload["reason"], "정지")


class FindClassNameTest(unittest.TestCase):
    def test_matches_by_id(self):
        objects = [FakeSceneObject("apple_17", "apple"), FakeSceneObject("cup_2", "cup")]
        self.assertEqual(find_class_name(objects, "cup_2"), "cup")

    def test_missing_id_returns_none(self):
        objects = [FakeSceneObject("apple_17", "apple")]
        self.assertIsNone(find_class_name(objects, "banana_1"))

    def test_empty_scene_returns_none(self):
        self.assertIsNone(find_class_name([], "apple_17"))


class FindSceneObjectTest(unittest.TestCase):
    def test_matches_by_id_and_returns_the_object(self):
        apple = FakeSceneObject("apple_17", "apple")
        objects = [apple, FakeSceneObject("cup_2", "cup")]
        self.assertIs(find_scene_object(objects, "apple_17"), apple)

    def test_missing_id_returns_none(self):
        objects = [FakeSceneObject("apple_17", "apple")]
        self.assertIsNone(find_scene_object(objects, "banana_1"))


class BboxCenterTest(unittest.TestCase):
    def test_center_of_a_simple_box(self):
        obj = FakeSceneObject("apple_17", "apple", bbox=(100.0, 200.0, 300.0, 400.0))
        self.assertEqual(bbox_center(obj), (200.0, 300.0))

    def test_zero_area_box_is_its_own_corner(self):
        obj = FakeSceneObject("apple_17", "apple", bbox=(50.0, 60.0, 50.0, 60.0))
        self.assertEqual(bbox_center(obj), (50.0, 60.0))


class ParsePickResultTest(unittest.TestCase):
    def test_valid_json_object(self):
        doc = parse_pick_result('{"request_id":"a1","result":"succeeded"}')
        self.assertEqual(doc, {"request_id": "a1", "result": "succeeded"})

    def test_malformed_json_returns_none(self):
        self.assertIsNone(parse_pick_result("not json"))

    def test_non_object_json_returns_none(self):
        self.assertIsNone(parse_pick_result("[1, 2, 3]"))
        self.assertIsNone(parse_pick_result('"just a string"'))


class ResultUpdateTest(unittest.TestCase):
    def test_accepted_is_not_terminal(self):
        update = result_update("accepted")
        self.assertFalse(update.terminal)
        self.assertEqual(update.status, "moving")
        self.assertEqual(update.last_result, "")

    def test_succeeded(self):
        update = result_update("succeeded")
        self.assertTrue(update.terminal)
        self.assertEqual(update.status, "idle")
        self.assertEqual(update.last_result, "succeeded")

    def test_failed(self):
        update = result_update("failed")
        self.assertEqual((update.terminal, update.status, update.last_result), (True, "idle", "failed"))

    def test_rejected(self):
        update = result_update("rejected")
        self.assertEqual((update.terminal, update.status, update.last_result), (True, "idle", "rejected"))

    def test_superseded_maps_to_rejected(self):
        """Per vla-integration.md #3-3's own mapping table -- not a guess."""
        update = result_update("superseded")
        self.assertEqual((update.terminal, update.status, update.last_result), (True, "idle", "rejected"))

    def test_unknown_result_fails_closed(self):
        """An undefined value must still release the one-action-in-flight
        gate -- silently ignoring it would strand the bridge forever."""
        update = result_update("something_new")
        self.assertEqual((update.terminal, update.status, update.last_result), (True, "idle", "failed"))


class FsmStateViewTest(unittest.TestCase):
    def test_holding_states_report_holding(self):
        for name in ("VERIFY", "LIFT", "PLACE", "PLACE_RETRY"):
            self.assertTrue(fsm_state_view(name).holding, name)

    def test_non_holding_state_does_not_report_holding(self):
        for name in ("APPROACH", "DESCEND", "HOME", "IDLE"):
            self.assertFalse(fsm_state_view(name).holding, name)

    def test_holding_set_matches_the_states_that_report_holding(self):
        self.assertEqual(FSM_HOLDING_STATES, {"VERIFY", "LIFT", "PLACE", "PLACE_RETRY"})

    def test_wait_approval_is_its_own_status(self):
        self.assertEqual(fsm_state_view("WAIT_APPROVAL").status, "waiting_approval")

    def test_known_states_map_to_documented_status_vocab(self):
        allowed = {"idle", "moving", "holding", "waiting_approval", "error"}
        for name in ("IDLE", "APPROACH", "LIFT", "WAIT_APPROVAL", "SAFE_STOP"):
            self.assertIn(fsm_state_view(name).status, allowed, name)

    def test_unknown_state_falls_through_to_moving_with_raw_label(self):
        view = fsm_state_view("SOME_NEW_STATE")
        self.assertEqual(view.status, "moving")
        self.assertEqual(view.label, "SOME_NEW_STATE")
        self.assertFalse(view.holding)

    def test_every_state_has_a_nonempty_label(self):
        for name in ("IDLE", "LIFT", "WAIT_APPROVAL", "HOME"):
            self.assertTrue(fsm_state_view(name).label.strip(), name)


if __name__ == "__main__":
    unittest.main()

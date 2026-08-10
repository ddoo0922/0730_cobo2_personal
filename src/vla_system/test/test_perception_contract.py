"""The pieces of the perception contract the LLM depends on being stable."""

import unittest

from vla_system.perception.detector import (
    mask_centroid,
    object_id,
    result_masks,
    result_to_detections,
)
from vla_system.perception.tracker import Detection, IoUTracker

import numpy as np


class ObjectIdTest(unittest.TestCase):
    def test_multiword_classes_lose_their_spaces(self):
        """The executor matches ids by exact string, so no spaces allowed."""
        self.assertEqual(object_id("wine glass", 4), "wine_glass_4")

    def test_case_and_padding_are_normalised(self):
        self.assertEqual(object_id("  Apple ", 17), "apple_17")

    def test_an_empty_class_still_produces_a_usable_handle(self):
        self.assertEqual(object_id("", 3), "object_3")

    def test_the_same_track_keeps_the_same_handle(self):
        self.assertEqual(object_id("apple", 17), object_id("apple", 17))


class TrackerIdentityTest(unittest.TestCase):
    def test_an_overlapping_box_keeps_its_id_across_frames(self):
        tracker = IoUTracker(iou_threshold=0.3)
        first = tracker.update([Detection(0, "apple", 0.9, (10, 10, 50, 50))], 0.0)
        second = tracker.update([Detection(0, "apple", 0.9, (12, 12, 52, 52))], 0.1)
        self.assertEqual(first[0].track_id, second[0].track_id)

    def test_a_disjoint_box_gets_a_new_id(self):
        tracker = IoUTracker(iou_threshold=0.3)
        first = tracker.update([Detection(0, "apple", 0.9, (10, 10, 50, 50))], 0.0)
        second = tracker.update([Detection(0, "apple", 0.9, (300, 300, 340, 340))], 0.1)
        self.assertNotEqual(first[0].track_id, second[0].track_id)

    def test_two_apples_get_two_ids(self):
        """Scenario 1 and 4 both hinge on this: same class, distinct handles."""
        tracker = IoUTracker()
        tracked = tracker.update(
            [
                Detection(0, "apple", 0.9, (10, 10, 50, 50)),
                Detection(0, "apple", 0.9, (200, 200, 240, 240)),
            ],
            0.0,
        )
        ids = {object_id(t.class_name, t.track_id) for t in tracked}
        self.assertEqual(len(ids), 2)


class MaskCarriesThroughTrackingTest(unittest.TestCase):
    def test_the_tracker_hands_the_mask_on_untouched(self):
        mask = np.ones((4, 4), dtype=bool)
        tracked = IoUTracker().update(
            [Detection(0, "apple", 0.9, (10, 10, 50, 50), mask=mask)], 0.0
        )
        self.assertIs(tracked[0].mask, mask)

    def test_a_detect_only_model_leaves_the_mask_empty(self):
        tracked = IoUTracker().update([Detection(0, "apple", 0.9, (10, 10, 50, 50))], 0.0)
        self.assertIsNone(tracked[0].mask)

    def test_identity_ignores_the_mask(self):
        """Two frames of the same object differ in mask, not in identity.

        The mask is excluded from the frozen dataclass comparison; if it were
        not, `==` would return a NumPy array and blow up on truth testing.
        """
        first = Detection(0, "apple", 0.9, (10, 10, 50, 50), mask=np.ones((2, 2), bool))
        second = Detection(0, "apple", 0.9, (10, 10, 50, 50), mask=np.zeros((2, 2), bool))
        self.assertEqual(first, second)


class MaskCentroidTest(unittest.TestCase):
    def test_the_centroid_is_the_centre_of_mass_in_xy_order(self):
        mask = np.zeros((10, 12), dtype=bool)
        mask[2:4, 6:10] = True
        self.assertEqual(mask_centroid(mask), (7.5, 2.5))

    def test_an_empty_mask_has_no_centroid(self):
        self.assertIsNone(mask_centroid(np.zeros((5, 5), dtype=bool)))


class _FakeTensor:
    """Stands in for a torch tensor: only .cpu() and len() are used."""

    def __init__(self, array):
        self._array = np.asarray(array)

    def cpu(self):
        return self

    def numpy(self):
        return self._array

    def tolist(self):
        return self._array.tolist()

    def __len__(self):
        return len(self._array)


class _FakeResult:
    def __init__(self, boxes, classes, masks, orig_shape, names):
        self.orig_shape = orig_shape
        self.names = names
        self.boxes = type(
            "Boxes",
            (),
            {
                "xyxy": _FakeTensor(boxes),
                "conf": _FakeTensor([0.9] * len(boxes)),
                "cls": _FakeTensor(classes),
            },
        )()
        self.masks = (
            None
            if masks is None
            else type("Masks", (), {"data": _FakeTensor(masks)})()
        )


class ResultMaskExtractionTest(unittest.TestCase):
    NAMES = {0: "apple", 1: "person"}

    def build(self, masks, orig_shape=(6, 8)):
        return _FakeResult(
            boxes=[[0, 0, 4, 4], [4, 0, 8, 4]],
            classes=[1, 0],  # person first, so filtering shifts the indices
            masks=masks,
            orig_shape=orig_shape,
            names=self.NAMES,
        )

    def test_masks_stay_paired_with_their_boxes_after_class_filtering(self):
        person_mask = np.zeros((6, 8), dtype=np.uint8)
        person_mask[0:2, 0:2] = 1
        apple_mask = np.zeros((6, 8), dtype=np.uint8)
        apple_mask[4:6, 6:8] = 1
        detections = result_to_detections(
            self.build([person_mask, apple_mask]),
            target_classes=set(),
            excluded_classes={"person"},
        )
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].class_name, "apple")
        # The surviving detection must carry the apple's mask, not the person's.
        np.testing.assert_array_equal(detections[0].mask, apple_mask.astype(bool))

    def test_a_mask_at_the_letterboxed_resolution_is_refused(self):
        """Silently rescaling would sample depth through the wrong pixels."""
        with self.assertRaises(ValueError):
            result_masks(self.build([np.zeros((640, 480), dtype=np.uint8)] * 2))

    def test_a_detect_only_result_reports_no_masks(self):
        self.assertIsNone(result_masks(self.build(None)))

    def test_an_empty_mask_batch_reports_no_masks(self):
        self.assertIsNone(result_masks(self.build([])))


if __name__ == "__main__":
    unittest.main()

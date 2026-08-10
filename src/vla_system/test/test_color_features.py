"""HSV colour naming, and what the instance mask changes about it.

The colour is what lets the agent tell "빨간 사과" from "노란 바나나" when both
are on the table, so the seven rainbow bands have to stay separable and the vote
has to come from the object rather than from whatever it is sitting on.
"""

import unittest

import cv2
import numpy as np

from vla_system.perception.color_features import (
    classify_detection_color,
    classify_masked_region,
)


def solid(hue, saturation=220, value=220, size=40):
    """One flat BGR patch built from an OpenCV hue (0-179)."""
    hsv = np.zeros((size, size, 3), dtype=np.uint8)
    hsv[:, :, 0] = hue
    hsv[:, :, 1] = saturation
    hsv[:, :, 2] = value
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def name_of(patch, mask=None):
    height, width = patch.shape[:2]
    name, _confidence, _source = classify_detection_color(
        patch, (0, 0, width, height), mask
    )
    return name


class RainbowBandTest(unittest.TestCase):
    """빨주노초파남보 -- each reference hue must land in its own band."""

    REFERENCE_HUES = {
        "red": 0,
        "orange": 15,
        "yellow": 30,
        "green": 60,
        "blue": 120,
        "indigo": 137,
        "violet": 150,
    }

    def test_each_rainbow_colour_is_named(self):
        for expected, hue in self.REFERENCE_HUES.items():
            with self.subTest(colour=expected, hue=hue):
                self.assertEqual(name_of(solid(hue)), expected)

    def test_blue_indigo_and_violet_are_three_separate_answers(self):
        """The split that replaced a single wide `purple` band."""
        named = {name_of(solid(self.REFERENCE_HUES[c])) for c in ("blue", "indigo", "violet")}
        self.assertEqual(named, {"blue", "indigo", "violet"})

    def test_red_wraps_around_the_top_of_the_hue_circle(self):
        self.assertEqual(name_of(solid(175)), "red")


class NonHueCategoryTest(unittest.TestCase):
    """Hues are not enough: most graspable objects are achromatic or brown."""

    def test_dark_orange_is_brown_not_orange(self):
        self.assertEqual(name_of(solid(15, value=90)), "brown")

    def test_unsaturated_pixels_become_black_white_or_gray(self):
        self.assertEqual(name_of(solid(0, saturation=0, value=30)), "black")
        self.assertEqual(name_of(solid(0, saturation=0, value=230)), "white")
        self.assertEqual(name_of(solid(0, saturation=0, value=120)), "gray")


class MaskedColourTest(unittest.TestCase):
    """A red apple on a white table is red, not white."""

    def setUp(self):
        self.image = np.full((60, 60, 3), 255, dtype=np.uint8)  # white table
        self.image[20:40, 20:40] = solid(0, size=20)  # red object
        self.mask = np.zeros((60, 60), dtype=bool)
        self.mask[20:40, 20:40] = True
        self.bbox = (0, 0, 60, 60)

    def test_the_box_is_outvoted_by_the_table(self):
        """The failure the mask exists to fix, pinned so it cannot come back."""
        name, _confidence, source = classify_detection_color(self.image, self.bbox)
        self.assertEqual(name, "white")
        self.assertEqual(source, "bbox_hsv")

    def test_the_mask_reports_the_object(self):
        name, confidence, source = classify_detection_color(
            self.image, self.bbox, self.mask
        )
        self.assertEqual(name, "red")
        self.assertEqual(source, "mask_hsv")
        # Confidence is the share of the object's own pixels, so a uniformly
        # coloured object scores near 1.0 instead of being diluted by the table.
        self.assertGreater(confidence, 0.99)

    def test_a_mask_too_small_to_judge_falls_back_to_the_box(self):
        """Better a background-tinted guess than `unknown`: colour only ever
        helps tell two objects apart, it never decides where the arm goes."""
        tiny = np.zeros((60, 60), dtype=bool)
        tiny[30:32, 30:33] = True  # 6 pixels, under MIN_SAMPLES
        name, _confidence, source = classify_detection_color(
            self.image, self.bbox, tiny
        )
        self.assertEqual(source, "bbox_hsv")
        self.assertEqual(name, "white")

    def test_an_empty_mask_falls_back_to_the_box(self):
        _name, _confidence, source = classify_detection_color(
            self.image, self.bbox, np.zeros((60, 60), dtype=bool)
        )
        self.assertEqual(source, "bbox_hsv")

    def test_a_mask_of_the_wrong_size_does_not_break_the_detection(self):
        _name, _confidence, source = classify_detection_color(
            self.image, self.bbox, np.ones((10, 10), dtype=bool)
        )
        self.assertEqual(source, "bbox_hsv")

    def test_the_helper_itself_refuses_a_mismatched_mask(self):
        with self.assertRaises(ValueError):
            classify_masked_region(self.image, self.bbox, np.ones((10, 10), dtype=bool))

    def test_a_box_clipped_away_to_nothing_is_unknown(self):
        name, confidence, source = classify_detection_color(
            self.image, (70, 70, 80, 80), self.mask
        )
        self.assertEqual((name, confidence, source), ("unknown", 0.0, "unknown"))


if __name__ == "__main__":
    unittest.main()

"""Dominant color estimation in HSV.

Given an instance mask, only the object's own pixels are counted, which is what
makes the colour trustworthy: a red apple on a white table no longer has half its
"colour" contributed by the table. Without a mask the centre of the bounding box
is sampled instead, which is the older and weaker approximation.

The chromatic bands are the seven rainbow colours -- 빨주노초파남보 -- as
``red``, ``orange``, ``yellow``, ``green``, ``blue``, ``indigo``, ``violet``.
Those are hue bands, so the categories that are not hues are kept alongside
them: ``brown`` (dark orange), and ``black`` / ``white`` / ``gray`` for pixels
with too little saturation to have a hue at all. Without them a black phone or a
white cup would have to be forced into a rainbow band or dropped entirely.

The result follows the VLA colour contract: an allowed colour name, a
confidence, and the source that produced it. Anything uncertain stays
``unknown``.
"""

import cv2
import numpy as np


# OpenCV hue is 0-179 (degrees halved), so red wraps around both ends. The
# reference hues these bands are cut around: red 0, orange 15, yellow 30,
# green 60, cyan 90, blue 120, indigo 137, violet/magenta 150.
HUE_RANGES = (
    ("red", 0, 9),
    ("orange", 10, 22),
    ("yellow", 23, 33),
    ("green", 34, 85),
    ("blue", 86, 126),
    ("indigo", 127, 142),
    ("violet", 143, 169),
    ("red", 170, 179),
)

MIN_SATURATION = 60
MIN_VALUE = 40
BLACK_VALUE = 55
WHITE_VALUE = 200
BROWN_VALUE = 110
CENTER_RATIO = 0.6
MIN_SAMPLES = 30


def _clamp_box(shape, x_min, y_min, x_max, y_max):
    """Clamp an xyxy box to the image, or None if nothing is left of it."""

    height, width = shape[:2]
    x_min = max(0, min(width - 1, int(x_min)))
    y_min = max(0, min(height - 1, int(y_min)))
    x_max = max(0, min(width, int(x_max)))
    y_max = max(0, min(height, int(y_max)))
    if x_max <= x_min or y_max <= y_min:
        return None
    return x_min, y_min, x_max, y_max


def _crop_center(image, x_min, y_min, x_max, y_max, ratio=CENTER_RATIO):
    box = _clamp_box(image.shape, x_min, y_min, x_max, y_max)
    if box is None:
        return None
    x_min, y_min, x_max, y_max = box

    box_width = x_max - x_min
    box_height = y_max - y_min
    inset_x = int(box_width * (1.0 - ratio) / 2.0)
    inset_y = int(box_height * (1.0 - ratio) / 2.0)
    cropped = image[
        y_min + inset_y : y_max - inset_y,
        x_min + inset_x : x_max - inset_x,
    ]
    if cropped.size == 0:
        return image[y_min:y_max, x_min:x_max]
    return cropped


def classify_hsv_pixels(hue, saturation, value):
    """Return (color_name, confidence) for flat HSV pixel arrays.

    The confidence is the winning colour's share of the pixels handed in, so a
    masked call reports the fraction of the *object* that is that colour.
    """

    total = hue.size
    if total < MIN_SAMPLES:
        return "unknown", 0.0

    lit = value >= MIN_VALUE
    chromatic = lit & (saturation >= MIN_SATURATION)
    counts = {}

    if int(np.count_nonzero(chromatic)):
        chromatic_hue = hue[chromatic]
        chromatic_value = value[chromatic]
        for name, low, high in HUE_RANGES:
            selected = (chromatic_hue >= low) & (chromatic_hue <= high)
            count = int(np.count_nonzero(selected))
            if not count:
                continue
            if name == "orange":
                # Brown is not a hue of its own, it is orange without the light.
                dark = int(np.count_nonzero(chromatic_value[selected] < BROWN_VALUE))
                counts["brown"] = counts.get("brown", 0) + dark
                counts["orange"] = counts.get("orange", 0) + count - dark
            else:
                counts[name] = counts.get(name, 0) + count

    achromatic = ~chromatic
    if np.count_nonzero(achromatic):
        achromatic_value = value[achromatic]
        counts["black"] = counts.get("black", 0) + int(
            np.count_nonzero(achromatic_value < BLACK_VALUE)
        )
        counts["white"] = counts.get("white", 0) + int(
            np.count_nonzero(achromatic_value > WHITE_VALUE)
        )
        counts["gray"] = counts.get("gray", 0) + int(
            np.count_nonzero(
                (achromatic_value >= BLACK_VALUE) & (achromatic_value <= WHITE_VALUE)
            )
        )

    counts = {name: count for name, count in counts.items() if count > 0}
    if not counts:
        return "unknown", 0.0

    best = max(counts, key=counts.get)
    return best, float(counts[best]) / float(total)


def classify_bgr_region(region):
    """Return (color_name, confidence) for one BGR crop, every pixel counted."""
    if region is None or region.size == 0:
        return "unknown", 0.0

    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    return classify_hsv_pixels(
        hsv[:, :, 0].reshape(-1),
        hsv[:, :, 1].reshape(-1),
        hsv[:, :, 2].reshape(-1),
    )


def classify_masked_region(image, bbox, mask):
    """Return (color_name, confidence) counting only pixels inside the mask.

    The conversion is done on the bounding-box crop rather than the whole frame,
    so a scene with a dozen objects does not pay for a dozen full-frame HSV
    conversions.
    """

    if image is None or mask is None:
        return "unknown", 0.0
    if mask.shape != image.shape[:2]:
        raise ValueError("mask must have the same height and width as image")

    box = _clamp_box(image.shape, *bbox)
    if box is None:
        return "unknown", 0.0
    x_min, y_min, x_max, y_max = box

    region = image[y_min:y_max, x_min:x_max]
    region_mask = mask[y_min:y_max, x_min:x_max].astype(bool)
    if region.size == 0 or not region_mask.any():
        return "unknown", 0.0

    selected = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)[region_mask]
    return classify_hsv_pixels(
        selected[:, 0], selected[:, 1], selected[:, 2]
    )


def classify_detection_color(image, bbox, mask=None):
    """Return (color_name, confidence, source) for one detection.

    Prefers the instance mask. Falls back to the bounding-box centre when there
    is no mask, or when the mask holds too few pixels to judge -- a small distant
    object is better described by a background-tinted guess than by ``unknown``,
    since the colour only ever helps the agent tell two objects apart and never
    decides where the arm goes.
    """

    if mask is not None and mask.shape == image.shape[:2]:
        name, confidence = classify_masked_region(image, bbox, mask)
        if name != "unknown":
            return name, confidence, "mask_hsv"

    region = _crop_center(image, *bbox)
    if region is None:
        return "unknown", 0.0, "unknown"
    name, confidence = classify_bgr_region(region)
    if name == "unknown":
        return "unknown", 0.0, "unknown"
    return name, confidence, "bbox_hsv"

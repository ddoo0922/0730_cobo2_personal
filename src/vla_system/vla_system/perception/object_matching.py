"""Deciding that the object under the wrist is the object the agent chose.

Two cameras see the same table from different places, and the arm has to be sure
they are talking about the same thing before it closes its fingers. The rule is
the obvious one: same class, and close enough in the robot's own frame.

Why the comparison is horizontal
--------------------------------
The two Z values do not measure the same thing and must not be compared as
though they did.

- The fixed webcam has no depth. Its Z is the *fitted tabletop plane* plus
  ``grasp_height_offset_m`` -- a standing guess about how high to close.
- The wrist RealSense measures the object's actual visible surface.

Those differ by roughly the offset (150 mm on this robot) even when both are
perfectly right. A full 3-D distance test would therefore reject every correct
match. So XY carries the decision, and Z gets a separate, deliberately loose gate
whose only job is to notice something on a different shelf.

Ambiguity is reported, not resolved
-----------------------------------
Two apples 3 cm apart can both fall inside the tolerance. Picking the nearer one
by a millimetre is a coin flip, and the cost of losing it is grasping the object
the user just said not to touch. ``ambiguous`` lets the caller refuse instead.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import math


@dataclass(frozen=True)
class WristCandidate:
    """One object the wrist camera currently sees, in base-frame metres."""

    handle: str
    class_name: str
    base_m: Sequence[float]


@dataclass(frozen=True)
class MatchResult:
    """Which candidate is the target, and how confident that reading is."""

    candidate: WristCandidate
    horizontal_distance_m: float
    vertical_gap_m: float
    runner_up_distance_m: Optional[float]
    ambiguous: bool


def horizontal_distance_m(first: Sequence[float], second: Sequence[float]) -> float:
    """Distance in the table plane only, ignoring height."""

    return math.hypot(float(first[0]) - float(second[0]), float(first[1]) - float(second[1]))


def match_target(
    target_class: str,
    target_base_m: Sequence[float],
    candidates: Iterable[WristCandidate],
    tolerance_m: float = 0.06,
    max_vertical_gap_m: float = 0.30,
    ambiguity_margin_m: float = 0.02,
) -> Optional[MatchResult]:
    """Find the wrist detection that is the agent's chosen object.

    ``tolerance_m`` is the horizontal budget. It has to cover the webcam's own
    error, which is dominated by parallax: a tall object's mask centre sits on
    its top face, so its mapped position drifts with height. 6 cm is roomy for a
    table of graspable objects and still far tighter than the spacing at which a
    user would call two things "that one".

    Returns None when nothing matches, which the caller should treat as "look
    again or give up" -- never as "grasp the nearest thing".
    """

    if tolerance_m <= 0.0:
        raise ValueError("tolerance_m must be greater than zero")
    if max_vertical_gap_m <= 0.0:
        raise ValueError("max_vertical_gap_m must be greater than zero")
    if ambiguity_margin_m < 0.0:
        raise ValueError("ambiguity_margin_m must be non-negative")

    scored = []
    for candidate in candidates:
        if candidate.class_name != target_class:
            continue
        distance = horizontal_distance_m(target_base_m, candidate.base_m)
        if distance > tolerance_m:
            continue
        vertical = abs(float(target_base_m[2]) - float(candidate.base_m[2]))
        if vertical > max_vertical_gap_m:
            continue
        scored.append((distance, vertical, candidate))

    if not scored:
        return None

    scored.sort(key=lambda item: item[0])
    distance, vertical, candidate = scored[0]
    runner_up = scored[1][0] if len(scored) > 1 else None
    return MatchResult(
        candidate=candidate,
        horizontal_distance_m=distance,
        vertical_gap_m=vertical,
        runner_up_distance_m=runner_up,
        ambiguous=runner_up is not None and (runner_up - distance) < ambiguity_margin_m,
    )

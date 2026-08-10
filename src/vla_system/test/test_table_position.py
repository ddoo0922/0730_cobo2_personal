"""The webcam-to-base position path the scene contract now rests on.

Since the RealSense moved to the wrist, an object's `position_base` comes from
the table homography instead of depth. Two things are easy to get wrong here and
both put the arm in the wrong place: the millimetre/metre boundary, and handing
out coordinates for pixels the calibration never covered.
"""

import json
import unittest

import numpy as np

from vla_system.perception.table_homography import (
    CalibrationError,
    build_table_calibration,
    load_table_calibration,
    table_point_from_pixel,
)


def flat_table_calibration(table_z_mm=40.0):
    """Pixels 100..500 x 100..400 over a level table at a known height."""
    pixels = [[100, 100], [500, 100], [500, 400], [100, 400]]
    xy = np.array([[300, -200], [700, -200], [700, 200], [300, 200]], dtype=float)
    robot = np.column_stack([xy, np.full(4, float(table_z_mm))])
    return build_table_calibration(pixels, robot)


class UnitBoundaryTest(unittest.TestCase):
    """The calibration speaks millimetres; the scene contract speaks metres."""

    def test_the_mapped_point_is_reported_in_metres(self):
        calibration = flat_table_calibration(table_z_mm=40.0)
        point = table_point_from_pixel(calibration, 300, 250)
        # Pixel centre maps to robot XY (500, 0) mm -> (0.5, 0.0) m.
        self.assertAlmostEqual(point.x_m, 0.5, places=6)
        self.assertAlmostEqual(point.y_m, 0.0, places=6)
        self.assertAlmostEqual(point.z_m, 0.040, places=6)

    def test_a_thousandfold_unit_error_would_be_caught(self):
        """Guards the specific failure: 500 mm published as 500 m, or 0.5 mm."""
        point = table_point_from_pixel(flat_table_calibration(), 300, 250)
        self.assertLess(abs(point.x_m), 2.0)
        self.assertGreater(abs(point.x_m), 0.05)

    def test_the_grasp_offset_is_added_in_metres(self):
        calibration = flat_table_calibration(table_z_mm=40.0)
        point = table_point_from_pixel(calibration, 300, 250, 0.02)
        self.assertAlmostEqual(point.z_m, 0.060, places=6)

    def test_a_tilted_table_changes_z_across_the_surface(self):
        pixels = [[100, 100], [500, 100], [500, 400], [100, 400]]
        xy = np.array([[300, -200], [700, -200], [700, 200], [300, 200]], dtype=float)
        z_mm = 0.01 * xy[:, 0] - 0.02 * xy[:, 1] + 700.0
        calibration = build_table_calibration(pixels, np.column_stack([xy, z_mm]))
        near = table_point_from_pixel(calibration, 150, 150)
        far = table_point_from_pixel(calibration, 450, 350)
        self.assertNotAlmostEqual(near.z_m, far.z_m, places=4)
        # Still plausible table heights in metres, not millimetres.
        for point in (near, far):
            self.assertLess(point.z_m, 1.5)


class OutsideTheCalibrationTest(unittest.TestCase):
    """Beyond the four measured corners the homography is extrapolating."""

    def test_a_pixel_inside_the_quad_is_marked_inside(self):
        point = table_point_from_pixel(flat_table_calibration(), 300, 250)
        self.assertTrue(point.inside_table)

    def test_a_pixel_outside_the_quad_is_marked_outside(self):
        """perception withholds the coordinate on this flag; the object is still
        reported so the agent can say it sees it."""
        point = table_point_from_pixel(flat_table_calibration(), 20, 20)
        self.assertFalse(point.inside_table)

    def test_an_outside_pixel_still_returns_numbers(self):
        """The flag is the gate, not a None: the node logs the extrapolated
        value while refusing to publish it."""
        point = table_point_from_pixel(flat_table_calibration(), 20, 20)
        self.assertTrue(np.isfinite([point.x_m, point.y_m, point.z_m]).all())


class CalibrationFileTest(unittest.TestCase):
    def setUp(self):
        self.payload = {
            "version": 1,
            "pixel_points": [[100, 100], [500, 100], [500, 400], [100, 400]],
            "robot_points_mm": [
                [300, -200, 40],
                [700, -200, 40],
                [700, 200, 40],
                [300, 200, 40],
            ],
        }

    def write(self, payload, name="cal.json"):
        import tempfile
        from pathlib import Path

        directory = Path(tempfile.mkdtemp())
        path = directory / name
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_a_saved_calibration_round_trips(self):
        calibration = load_table_calibration(self.write(self.payload))
        point = table_point_from_pixel(calibration, 300, 250)
        self.assertAlmostEqual(point.x_m, 0.5, places=6)
        self.assertAlmostEqual(point.z_m, 0.040, places=6)

    def test_a_missing_file_raises_rather_than_returning_a_default(self):
        with self.assertRaises(FileNotFoundError):
            load_table_calibration("/nonexistent/table_calibration.json")

    def test_a_file_without_the_point_correspondences_is_rejected(self):
        with self.assertRaises(CalibrationError):
            load_table_calibration(self.write({"version": 1}))

    def test_a_truncated_point_list_is_rejected(self):
        payload = dict(self.payload)
        payload["pixel_points"] = payload["pixel_points"][:3]
        with self.assertRaises(CalibrationError):
            load_table_calibration(self.write(payload))

    def test_a_resolution_mismatch_is_refused(self):
        """Raw pixels do not survive a resolution change: the same click means a
        different table point at 640x480 than at 1280x720."""
        payload = dict(self.payload, image_size=[1280, 720])
        path = self.write(payload)
        load_table_calibration(path, (1280, 720))  # matching: fine
        with self.assertRaises(CalibrationError):
            load_table_calibration(path, (640, 480))

    def test_a_file_without_a_recorded_resolution_is_still_accepted(self):
        """Calibrations saved before image_size existed cannot be checked, but
        refusing them outright would strand a working setup."""
        calibration = load_table_calibration(self.write(self.payload), (640, 480))
        self.assertIsNotNone(calibration)

    def test_a_degenerate_saved_calibration_is_rejected(self):
        """A hand-edited or collapsed file must fail here, not aim the arm."""
        payload = dict(self.payload)
        payload["pixel_points"] = [[0, 0], [1, 0], [1, 1], [0, 1]]
        with self.assertRaises(CalibrationError):
            load_table_calibration(self.write(payload))


if __name__ == "__main__":
    unittest.main()

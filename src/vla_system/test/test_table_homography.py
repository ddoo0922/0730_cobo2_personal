import numpy as np

from vla_system.perception.table_homography import (
    build_table_calibration,
    point_inside_calibrated_polygon,
)


def test_pixel_to_robot_xy_and_table_z():
    pixels = [[100, 100], [500, 100], [500, 400], [100, 400]]
    # Slightly tilted table: z = 0.01*x - 0.02*y + 700
    xy = np.array([[300, -200], [700, -200], [700, 200], [300, 200]], dtype=float)
    z = 0.01 * xy[:, 0] - 0.02 * xy[:, 1] + 700.0
    robot = np.column_stack([xy, z])

    calibration = build_table_calibration(pixels, robot)

    x, y, target_z = calibration.approach_target_mm(300, 250, 150.0)
    assert np.allclose([x, y], [500.0, 0.0], atol=1e-4)
    assert np.isclose(target_z, 855.0, atol=1e-4)
    assert point_inside_calibrated_polygon(calibration, 300, 250)
    assert not point_inside_calibrated_polygon(calibration, 20, 20)

#!/usr/bin/env python3
"""Interactive table homography + click-to-approach test for M0609.

Workflow
--------
1. Jog the robot TCP to table P1..P4 and press SPACE for each point.
2. Click the exact same P1..P4 locations in the webcam image.
3. The node computes pixel -> robot-base XY homography and a table Z plane.
4. In TEST mode, left-click anywhere inside the calibrated table polygon.
   The target is [X, Y, Z_table(X,Y) + z_offset_mm].
5. With motion_enabled:=true the M0609 moves there while keeping its current
   TCP orientation. With motion_enabled:=false only the target is printed.

P1..P4 must be ordered around the table perimeter consistently (CW or CCW).
Press X during a real move to request SSTOP through /<robot_id>/motion/move_stop.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from vla_system.perception.table_homography import (
    CalibrationError,
    TableCalibration,
    build_table_calibration,
    point_inside_calibrated_polygon,
    reprojection_errors_mm,
)

# From dsr_msgs2/srv/MoveStop.srv.
DR_SSTOP = 2
DR_STATE_IDLE = 0


class DoosanClickArm:
    """Small Doosan driver dedicated to this calibration utility.

    DSR_ROBOT2 spins DR_init.__dsr__node internally. We therefore keep a
    separate private ROS node for DSR calls and make sure only one worker at a
    time touches this object.
    """

    def __init__(
        self,
        robot_id: str,
        robot_model: str,
        velocity: float,
        acceleration: float,
        poll_interval_s: float,
        motion_start_grace_s: float,
        motion_timeout_s: float,
    ):
        import DR_init

        setattr(DR_init, "__dsr__id", robot_id)
        setattr(DR_init, "__dsr__model", robot_model)
        self.node = rclpy.create_node("table_homography_dsr_driver", namespace=robot_id)
        setattr(DR_init, "__dsr__node", self.node)

        from DSR_ROBOT2 import DR_MV_MOD_ABS, amovel, check_motion, get_current_posx
        from DR_common2 import posx

        self.absolute_mode = DR_MV_MOD_ABS
        self.amovel = amovel
        self.check_motion = check_motion
        self.get_current_posx = get_current_posx
        self.posx = posx
        self.velocity = float(velocity)
        self.acceleration = float(acceleration)
        self.poll_interval_s = float(poll_interval_s)
        self.motion_start_grace_s = float(motion_start_grace_s)
        self.motion_timeout_s = float(motion_timeout_s)

    def current_pose_mm(self) -> list[float]:
        current = self.get_current_posx()
        if current is None or current[0] is None or len(current[0]) < 6:
            raise RuntimeError("get_current_posx returned no valid TCP pose")
        return [float(v) for v in current[0][:6]]

    def move_xyz_keep_orientation(self, x_mm: float, y_mm: float, z_mm: float) -> None:
        current = self.current_pose_mm()
        target = self.posx([x_mm, y_mm, z_mm, *current[3:6]])
        result = self.amovel(
            target,
            vel=self.velocity,
            acc=self.acceleration,
            mod=self.absolute_mode,
        )
        if result is not None and result < 0:
            raise RuntimeError("amovel rejected target")

        # Avoid treating the pre-motion IDLE as completion.
        grace_deadline = time.monotonic() + self.motion_start_grace_s
        while time.monotonic() < grace_deadline:
            if self.check_motion() != DR_STATE_IDLE:
                break
            time.sleep(self.poll_interval_s)

        deadline = time.monotonic() + self.motion_timeout_s
        while True:
            status = self.check_motion()
            if status == DR_STATE_IDLE:
                return
            if status < 0:
                raise RuntimeError("check_motion failed")
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"motion timed out after {self.motion_timeout_s:.1f}s"
                )
            time.sleep(self.poll_interval_s)

    def close(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:
            pass


class TableHomographyTestNode(Node):
    WINDOW = "VLA Table Homography Test"

    def __init__(self):
        super().__init__("table_homography_test")

        # Camera/UI
        self.declare_parameter("webcam_device", "/dev/video8")
        self.declare_parameter("webcam_width", 1280)
        self.declare_parameter("webcam_height", 720)
        self.declare_parameter("webcam_fps", 30.0)
        self.declare_parameter("z_offset_mm", 150.0)
        self.declare_parameter(
            "calibration_file", os.path.expanduser("~/.ros/vla_table_homography.json")
        )
        self.declare_parameter("load_calibration_on_start", False)
        self.declare_parameter("reject_clicks_outside_table", True)

        # Robot
        self.declare_parameter("motion_enabled", False)
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "m0609")
        self.declare_parameter("velocity", 30.0)
        self.declare_parameter("acceleration", 30.0)
        self.declare_parameter("poll_interval_s", 0.02)
        self.declare_parameter("motion_start_grace_s", 0.3)
        self.declare_parameter("motion_timeout_s", 30.0)
        self.declare_parameter("workspace_min_x_mm", 200.0)
        self.declare_parameter("workspace_max_x_mm", 900.0)
        self.declare_parameter("workspace_min_y_mm", -600.0)
        self.declare_parameter("workspace_max_y_mm", 600.0)
        self.declare_parameter("workspace_min_z_mm", 20.0)
        self.declare_parameter("workspace_max_z_mm", 800.0)

        self.motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self.robot_id = str(self.get_parameter("robot_id").value)
        self.z_offset_mm = float(self.get_parameter("z_offset_mm").value)
        self.reject_outside = bool(
            self.get_parameter("reject_clicks_outside_table").value
        )
        self.calibration_path = Path(
            os.path.expanduser(str(self.get_parameter("calibration_file").value))
        )

        self.workspace = {
            "x": (
                float(self.get_parameter("workspace_min_x_mm").value),
                float(self.get_parameter("workspace_max_x_mm").value),
            ),
            "y": (
                float(self.get_parameter("workspace_min_y_mm").value),
                float(self.get_parameter("workspace_max_y_mm").value),
            ),
            "z": (
                float(self.get_parameter("workspace_min_z_mm").value),
                float(self.get_parameter("workspace_max_z_mm").value),
            ),
        }

        self.robot_points_mm: list[list[float]] = []
        self.pixel_points: list[list[float]] = []
        self.calibration: Optional[TableCalibration] = None
        self.last_click: Optional[tuple[int, int]] = None
        self.last_target_mm: Optional[tuple[float, float, float]] = None
        self.motion_thread: Optional[threading.Thread] = None
        self.motion_lock = threading.Lock()
        self.motion_error = ""
        self.stop_requested = threading.Event()

        self.arm = DoosanClickArm(
            robot_id=self.robot_id,
            robot_model=str(self.get_parameter("robot_model").value),
            velocity=float(self.get_parameter("velocity").value),
            acceleration=float(self.get_parameter("acceleration").value),
            poll_interval_s=float(self.get_parameter("poll_interval_s").value),
            motion_start_grace_s=float(
                self.get_parameter("motion_start_grace_s").value
            ),
            motion_timeout_s=float(self.get_parameter("motion_timeout_s").value),
        )

        self.stop_client = None
        if self.motion_enabled:
            try:
                from dsr_msgs2.srv import MoveStop

                self.move_stop_type = MoveStop
                self.stop_client = self.create_client(
                    MoveStop, f"/{self.robot_id}/motion/move_stop"
                )
            except ImportError as exc:
                self.get_logger().warning(f"MoveStop service unavailable: {exc}")

        self.capture = self._open_camera()
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.WINDOW, self._mouse_callback)

        if bool(self.get_parameter("load_calibration_on_start").value):
            self._load_calibration()

        if self.motion_enabled:
            self.get_logger().warning(
                "REAL MOTION ENABLED: a left click in TEST mode will move the M0609"
            )
        else:
            self.get_logger().warning(
                "DRY RUN: clicks compute targets only. Use -p motion_enabled:=true to move."
            )
        self._print_stage_help()

    # ---------------------------------------------------------------- camera

    def _open_camera(self):
        raw = str(self.get_parameter("webcam_device").value).strip()
        source = int(raw) if raw.isdigit() else raw
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open webcam: {raw}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("webcam_width").value))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("webcam_height").value))
        cap.set(cv2.CAP_PROP_FPS, float(self.get_parameter("webcam_fps").value))
        return cap

    # --------------------------------------------------------------- workflow

    @property
    def stage(self) -> str:
        if len(self.robot_points_mm) < 4:
            return "ROBOT_POINTS"
        if len(self.pixel_points) < 4 or self.calibration is None:
            return "WEBCAM_POINTS"
        return "TEST"

    def _print_stage_help(self) -> None:
        if self.stage == "ROBOT_POINTS":
            self.get_logger().info(
                "[1/3] Jog TCP to P1..P4 around the table. Press SPACE at each contact point."
            )
        elif self.stage == "WEBCAM_POINTS":
            self.get_logger().info(
                "[2/3] Click the SAME P1..P4 physical points in the webcam, same order."
            )
        else:
            self.get_logger().info(
                f"[3/3] Click inside the table. Target Z = fitted table Z + {self.z_offset_mm:.1f} mm."
            )

    def record_robot_point(self) -> None:
        if len(self.robot_points_mm) >= 4:
            return
        pose = self.arm.current_pose_mm()
        xyz = pose[:3]
        self.robot_points_mm.append(xyz)
        index = len(self.robot_points_mm)
        self.get_logger().info(
            f"Robot P{index}: X={xyz[0]:.2f}, Y={xyz[1]:.2f}, Z={xyz[2]:.2f} mm"
        )
        if index == 4:
            self._print_stage_help()

    def _mouse_callback(self, event, x, y, flags, userdata) -> None:
        del flags, userdata
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.stage == "ROBOT_POINTS":
            self.get_logger().warning(
                "Record all four robot TCP points first with SPACE."
            )
            return

        if self.stage == "WEBCAM_POINTS":
            self.pixel_points.append([float(x), float(y)])
            index = len(self.pixel_points)
            self.get_logger().info(f"Webcam P{index}: u={x}, v={y}")
            if index == 4:
                self._finish_calibration()
            return

        self.last_click = (int(x), int(y))
        self._handle_test_click(float(x), float(y))

    def _finish_calibration(self) -> None:
        try:
            self.calibration = build_table_calibration(
                self.pixel_points, self.robot_points_mm
            )
        except CalibrationError as exc:
            self.get_logger().error(f"Calibration failed: {exc}")
            self.pixel_points.clear()
            self.calibration = None
            return

        errors = reprojection_errors_mm(self.calibration)
        a, b, c = self.calibration.plane_z_coefficients
        self.get_logger().info(
            "Homography ready. Calibration-point XY reprojection error [mm]: "
            + ", ".join(f"{v:.3f}" for v in errors)
        )
        self.get_logger().info(
            f"Table plane: Z_mm = {a:.8f}*X + {b:.8f}*Y + {c:.3f}"
        )
        self._save_calibration()
        self._print_stage_help()

    def _handle_test_click(self, u: float, v: float) -> None:
        assert self.calibration is not None
        if self.reject_outside and not point_inside_calibrated_polygon(
            self.calibration, u, v
        ):
            self.get_logger().warning("Click rejected: outside calibrated table polygon")
            return

        target = self.calibration.approach_target_mm(u, v, self.z_offset_mm)
        self.last_target_mm = target
        x_mm, y_mm, z_mm = target
        self.get_logger().info(
            f"Click ({u:.0f},{v:.0f}) -> base target "
            f"X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f} mm "
            f"(table + {self.z_offset_mm:.1f} mm)"
        )

        if not self._target_inside_workspace(target):
            self.get_logger().error("Target rejected: outside configured robot workspace")
            return
        if not self.motion_enabled:
            return

        with self.motion_lock:
            if self.motion_thread is not None and self.motion_thread.is_alive():
                self.get_logger().warning("Robot is already moving; click ignored")
                return
            self.motion_error = ""
            self.stop_requested.clear()
            self.motion_thread = threading.Thread(
                target=self._motion_worker,
                # One argument, not three: _motion_worker takes the whole
                # (x, y, z) tuple. `args=target` would spread it into three
                # positional arguments and raise TypeError inside the thread.
                args=(target,),
                name="table-homography-motion",
                daemon=True,
            )
            self.motion_thread.start()

    def _motion_worker(self, target) -> None:
        try:
            x_mm, y_mm, z_mm = target
            self.get_logger().warning(
                f"MOVE -> X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f} mm"
            )
            self.arm.move_xyz_keep_orientation(x_mm, y_mm, z_mm)
            if self.stop_requested.is_set():
                raise RuntimeError("motion stopped by user")
            self.get_logger().info("Move complete")
        except Exception as exc:
            self.motion_error = str(exc)
            self.get_logger().error(f"Move failed/stopped: {exc}")

    def request_stop(self) -> None:
        if not self.motion_enabled:
            self.get_logger().info("Dry-run: no hardware motion to stop")
            return
        if self.stop_client is None:
            self.get_logger().error("MoveStop client is unavailable")
            return
        self.stop_requested.set()
        request = self.move_stop_type.Request()
        request.stop_mode = DR_SSTOP
        self.stop_client.call_async(request)
        self.get_logger().warning("SSTOP requested")

    def _target_inside_workspace(self, target) -> bool:
        for axis, value in zip("xyz", target):
            low, high = self.workspace[axis]
            if not low <= float(value) <= high:
                self.get_logger().error(
                    f"{axis.upper()}={value:.1f} mm outside [{low:.1f}, {high:.1f}] mm"
                )
                return False
        return True

    # ------------------------------------------------------------ persistence

    def _save_calibration(self) -> None:
        if self.calibration is None:
            return
        self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "units": {"robot": "mm", "image": "pixel"},
            # The resolution these pixels were clicked at. vla_perception refuses
            # the calibration if it opens the webcam at a different size, since
            # raw pixel coordinates do not survive a resolution change.
            "image_size": [
                int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            ],
            "robot_points_mm": self.calibration.robot_points_mm.tolist(),
            "pixel_points": self.calibration.pixel_points.tolist(),
            "homography_pixel_to_base_xy_mm": self.calibration.homography.tolist(),
            "table_plane_z_coefficients": self.calibration.plane_z_coefficients.tolist(),
            "z_offset_mm": self.z_offset_mm,
        }
        self.calibration_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        self.get_logger().info(f"Calibration saved: {self.calibration_path}")

    def _load_calibration(self) -> None:
        if not self.calibration_path.exists():
            self.get_logger().warning(
                f"Calibration file not found: {self.calibration_path}"
            )
            return
        try:
            payload = json.loads(self.calibration_path.read_text(encoding="utf-8"))
            self.robot_points_mm = [
                [float(v) for v in point] for point in payload["robot_points_mm"]
            ]
            self.pixel_points = [
                [float(v) for v in point] for point in payload["pixel_points"]
            ]
            self.calibration = build_table_calibration(
                self.pixel_points, self.robot_points_mm
            )
            self.get_logger().info(f"Calibration loaded: {self.calibration_path}")
        except Exception as exc:
            self.get_logger().error(f"Could not load calibration: {exc}")
            self.reset_calibration()

    def reset_calibration(self) -> None:
        if self.motion_thread is not None and self.motion_thread.is_alive():
            self.get_logger().warning("Cannot reset calibration while robot is moving")
            return
        self.robot_points_mm.clear()
        self.pixel_points.clear()
        self.calibration = None
        self.last_click = None
        self.last_target_mm = None
        self.get_logger().warning("Calibration reset")
        self._print_stage_help()

    def undo(self) -> None:
        if self.stage == "TEST":
            self.get_logger().info("TEST mode: press R to reset the calibration")
            return
        if self.stage == "WEBCAM_POINTS" and self.pixel_points:
            removed = self.pixel_points.pop()
            self.get_logger().info(f"Removed webcam point {removed}")
            return
        if self.robot_points_mm:
            removed = self.robot_points_mm.pop()
            self.get_logger().info(f"Removed robot point {removed}")

    # ------------------------------------------------------------------ draw

    def _draw(self, frame: np.ndarray) -> np.ndarray:
        image = frame.copy()
        h, w = image.shape[:2]

        # Calibration points and polygon.
        if self.pixel_points:
            for i, point in enumerate(self.pixel_points):
                px = (int(round(point[0])), int(round(point[1])))
                cv2.circle(image, px, 7, (0, 255, 255), -1)
                cv2.putText(
                    image,
                    f"P{i+1}",
                    (px[0] + 8, px[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            if len(self.pixel_points) == 4:
                contour = np.asarray(self.pixel_points, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(image, [contour], True, (0, 255, 255), 2)

        if self.last_click is not None:
            cv2.drawMarker(
                image,
                self.last_click,
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=24,
                thickness=2,
            )

        stage_text = {
            "ROBOT_POINTS": f"STEP 1/3: TCP table points {len(self.robot_points_mm)}/4 | SPACE=record",
            "WEBCAM_POINTS": f"STEP 2/3: webcam points {len(self.pixel_points)}/4 | LEFT CLICK=P1..P4",
            "TEST": "STEP 3/3: LEFT CLICK=target | X=stop | R=reset | Q=quit",
        }[self.stage]
        cv2.rectangle(image, (0, 0), (w, 86), (0, 0, 0), -1)
        cv2.putText(
            image,
            stage_text,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        mode = "MOTION ENABLED" if self.motion_enabled else "DRY RUN"
        cv2.putText(
            image,
            f"{mode} | approach Z = table + {self.z_offset_mm:.1f} mm",
            (15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 200, 255) if self.motion_enabled else (180, 180, 180),
            2,
            cv2.LINE_AA,
        )

        if self.last_target_mm is not None:
            x_mm, y_mm, z_mm = self.last_target_mm
            cv2.putText(
                image,
                f"Target base [mm] X={x_mm:.1f} Y={y_mm:.1f} Z={z_mm:.1f}",
                (15, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return image

    # ------------------------------------------------------------------ loop

    def run(self) -> None:
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            ok, frame = self.capture.read()
            if not ok or frame is None:
                self.get_logger().error("Webcam frame read failed")
                time.sleep(0.05)
                continue

            cv2.imshow(self.WINDOW, self._draw(frame))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                if self.stage == "ROBOT_POINTS":
                    try:
                        self.record_robot_point()
                    except Exception as exc:
                        self.get_logger().error(f"Could not read TCP pose: {exc}")
            elif key in (ord("u"), 8):
                self.undo()
            elif key == ord("r"):
                self.reset_calibration()
            elif key == ord("s"):
                self._save_calibration()
            elif key == ord("l"):
                self._load_calibration()
                self._print_stage_help()
            elif key == ord("x"):
                self.request_stop()

    def close(self) -> None:
        if self.motion_thread is not None and self.motion_thread.is_alive():
            self.request_stop()
            self.motion_thread.join(timeout=2.0)
        try:
            self.capture.release()
        except Exception:
            pass
        cv2.destroyAllWindows()
        self.arm.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TableHomographyTestNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

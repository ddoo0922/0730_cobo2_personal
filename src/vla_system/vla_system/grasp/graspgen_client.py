"""The boundary between this system and GraspGenX.

Everything that needs a GPU, model weights, or the ``graspgenx`` package lives
here. The geometry that interprets what comes back lives in ``grasp.poses`` and
is tested without any of that.

Measured cost on the RTX 4060 Laptop (8 GB), ``onrobot_RG2``:

- load: about 15 s, one time
- inference: about 1.6 s per object
- VRAM: 1163 MB peak, alongside YOLO-seg's ~300 MB

The 1.6 s matters for the stop path. It is long enough that a "정지" can arrive
mid-inference, so whoever calls this has to re-check the stop epoch before acting
on the result -- the same guarantee the agent already gives around LLM calls.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from time import perf_counter
from typing import Optional, Sequence

import numpy as np

from vla_system.grasp.poses import RG2_FINGERTIP_M, build_candidates

# GraspGenX consumes and emits metres; our wrist geometry works in millimetres.
M_PER_MM = 1.0 / 1000.0


class GraspGenUnavailable(RuntimeError):
    """GraspGenX could not be loaded. The caller should fall back."""


def default_repo_path() -> Optional[Path]:
    """Where GraspGenX lives, if it can be found without being told."""

    override = os.environ.get("GRASPGENX_ROOT")
    if override:
        return Path(override).expanduser()
    try:
        import graspgenx

        # <repo>/graspgenx/__init__.py -> <repo>
        return Path(graspgenx.__file__).resolve().parent.parent
    except Exception:
        return None


def read_gripper_fingertip(repo_path, gripper_name: str) -> Sequence[float]:
    """The gripper's own fingertip offset, from its shipped config.json.

    Read rather than hardcoded because it is the number that decides where the
    fingers close: for ``onrobot_RG2`` it is 180 mm along the gripper's +Z, and
    getting it from the same file the model was conditioned on removes the
    chance of the two disagreeing.
    """

    if repo_path is None:
        return RG2_FINGERTIP_M
    candidates = [
        Path(repo_path)
        / "ext/gripper_descriptions/gripper_descriptions/assets/x_grippers"
        / gripper_name
        / "config.json",
        Path(repo_path) / "assets/x_grippers" / gripper_name / "config.json",
        Path(repo_path) / "assets/proc_grippers" / gripper_name / "config.json",
    ]
    for path in candidates:
        if path.is_file():
            payload = json.loads(path.read_text(encoding="utf-8"))
            fingertip = payload.get("fingertip")
            if isinstance(fingertip, (list, tuple)) and len(fingertip) == 3:
                return tuple(float(v) for v in fingertip)
    return RG2_FINGERTIP_M


class GraspGenClient:
    """Loads GraspGenX once and generates grasps for a segmented point cloud."""

    def __init__(
        self,
        gripper_name: str = "onrobot_RG2",
        repo_path=None,
        num_grasps: int = 200,
        topk_num_grasps: int = 20,
        grasp_threshold: float = -1.0,
        logger=None,
    ):
        self.gripper_name = gripper_name
        self.num_grasps = int(num_grasps)
        self.topk_num_grasps = int(topk_num_grasps)
        self.grasp_threshold = float(grasp_threshold)
        self._log = logger

        self.repo_path = Path(repo_path).expanduser() if repo_path else default_repo_path()
        if self.repo_path is None or not self.repo_path.is_dir():
            raise GraspGenUnavailable(
                "GraspGenX repository not found. Set GRASPGENX_ROOT or the "
                "graspgen_repo parameter."
            )

        started = perf_counter()
        try:
            from graspgenx import get_checkpoints_version_dir
            from graspgenx.grasp_server import GraspGenXSampler
            from graspgenx.utils.checkpoint_io import load_model_cfg
        except Exception as exc:
            raise GraspGenUnavailable(f"cannot import graspgenx: {exc}") from exc

        try:
            checkpoints = Path(get_checkpoints_version_dir())
            cfg = load_model_cfg(str(checkpoints / "gen"), str(checkpoints / "dis"))
            self._sampler_class = GraspGenXSampler
            self._sampler = GraspGenXSampler(
                cfg,
                gripper_name=gripper_name,
                assets_dir=str(self.repo_path / "assets"),
            )
        except Exception as exc:
            raise GraspGenUnavailable(f"cannot load GraspGenX model: {exc}") from exc

        self.fingertip_m = read_gripper_fingertip(self.repo_path, gripper_name)
        self.load_s = perf_counter() - started
        self._info(
            f"GraspGenX ready in {self.load_s:.1f}s | gripper={gripper_name} "
            f"fingertip={tuple(round(v, 4) for v in self.fingertip_m)}m"
        )

    def _info(self, message: str) -> None:
        if self._log is not None:
            self._log.info(message)

    def generate(self, points_mm) -> list:
        """Grasp candidates for one segmented object cloud.

        ``points_mm`` is (N, 3) in the **camera** frame in millimetres, as
        ``wrist_geometry.mask_to_camera_cloud`` produces it. It is converted to
        metres here because that is what GraspGenX was trained on -- handing it
        millimetres yields a cloud a thousand times too large and grasps that
        look plausible in shape and are nonsense in scale.

        The returned candidates are in the **camera** frame, in metres. Moving
        them to the base frame needs the hand-eye chain and the arm's pose at the
        instant the depth frame was taken, which only the caller knows.
        """

        cloud = np.asarray(points_mm, dtype=np.float32)
        if cloud.ndim != 2 or cloud.shape[1] != 3:
            raise ValueError(f"expected (N, 3) points, got {cloud.shape}")
        if len(cloud) == 0:
            return []

        started = perf_counter()
        poses, confidences = self._sampler_class.run_inference(
            cloud * M_PER_MM,
            self._sampler,
            grasp_threshold=self.grasp_threshold,
            num_grasps=self.num_grasps,
            topk_num_grasps=self.topk_num_grasps,
        )
        elapsed = perf_counter() - started

        poses = poses.detach().cpu().numpy()
        confidences = confidences.detach().cpu().numpy()
        candidates = build_candidates(poses, confidences, self.fingertip_m)
        self._info(
            f"GraspGenX: {len(cloud)} points -> {len(candidates)} grasps "
            f"in {elapsed * 1000:.0f}ms"
            + (
                f" | best confidence {candidates[0].confidence:.3f}"
                if candidates
                else " | no candidates"
            )
        )
        return candidates

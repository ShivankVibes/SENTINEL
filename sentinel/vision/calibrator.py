"""Camera calibration and metric floor coordinate mapping using 4-point homography."""

import os
import json
import logging
from typing import List, Tuple, Optional
import cv2
import numpy as np

logger = logging.getLogger("sentinel.calibrator")


class ArenaCalibrator:
    """Manages 4-point perspective transformation between camera pixels and arena floor coordinates (cm)."""

    def __init__(
        self,
        arena_width_cm: float = 200.0,
        arena_height_cm: float = 150.0,
        config_path: Optional[str] = None,
    ):
        self.arena_width_cm = float(arena_width_cm)
        self.arena_height_cm = float(arena_height_cm)
        self.config_path = config_path or "data/calibration.json"

        # 4 source points in camera image coordinates: [Top-Left, Top-Right, Bottom-Right, Bottom-Left]
        self.source_points: Optional[np.ndarray] = None
        # 4 destination points in metric ground coordinates (cm):
        # (0, 0) is Top-Left, (W, 0) is Top-Right, (W, H) is Bottom-Right, (0, H) is Bottom-Left
        self.dest_points = np.array(
            [
                [0.0, 0.0],
                [self.arena_width_cm, 0.0],
                [self.arena_width_cm, self.arena_height_cm],
                [0.0, self.arena_height_cm],
            ],
            dtype=np.float32,
        )

        self.homography_matrix: Optional[np.ndarray] = None
        self.inv_homography_matrix: Optional[np.ndarray] = None
        self.is_calibrated = False

        self.load_calibration()

    def set_calibration_points(self, points: List[Tuple[float, float]]) -> bool:
        """Set the 4 corner pixel coordinates and compute the homography matrix.
        
        Points order must be: [Top-Left, Top-Right, Bottom-Right, Bottom-Left].
        """
        if len(points) != 4:
            raise ValueError(f"Exactly 4 points required, received {len(points)}")

        src = np.array(points, dtype=np.float32)
        H, status = cv2.findHomography(src, self.dest_points)
        if H is None:
            logger.error("Failed to compute homography matrix from given points.")
            return False

        inv_H = np.linalg.inv(H)
        self.source_points = src
        self.homography_matrix = H
        self.inv_homography_matrix = inv_H
        self.is_calibrated = True

        logger.info("Homography calibration matrix successfully computed.")
        self.save_calibration()
        return True

    def pixel_to_floor(self, u: float, v: float) -> Tuple[float, float]:
        """Transform image pixel coordinates (u, v) to physical floor coordinates (x_cm, y_cm)."""
        if not self.is_calibrated or self.homography_matrix is None:
            # Fallback direct scaling if uncalibrated
            scale_x = self.arena_width_cm / 1280.0
            scale_y = self.arena_height_cm / 720.0
            return round(u * scale_x, 1), round(v * scale_y, 1)

        pt = np.array([[[u, v]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.homography_matrix)
        x_cm, y_cm = transformed[0][0]
        return round(float(x_cm), 1), round(float(y_cm), 1)

    def floor_to_pixel(self, x_cm: float, y_cm: float) -> Tuple[int, int]:
        """Transform physical floor coordinates (x_cm, y_cm) to image pixel coordinates (u, v)."""
        if not self.is_calibrated or self.inv_homography_matrix is None:
            scale_x = 1280.0 / self.arena_width_cm
            scale_y = 720.0 / self.arena_height_cm
            return int(x_cm * scale_x), int(y_cm * scale_y)

        pt = np.array([[[x_cm, y_cm]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.inv_homography_matrix)
        u, v = transformed[0][0]
        return int(round(float(u))), int(round(float(v)))

    def save_calibration(self):
        """Save calibration data to JSON file."""
        if not self.is_calibrated or self.source_points is None:
            return

        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        data = {
            "arena_width_cm": self.arena_width_cm,
            "arena_height_cm": self.arena_height_cm,
            "source_points": self.source_points.tolist(),
            "homography_matrix": self.homography_matrix.tolist() if self.homography_matrix is not None else None,
        }
        try:
            with open(self.config_path, "w") as f:
                json.dump(data, f, indent=2)
            logger.info(f"Calibration saved to {self.config_path}")
        except Exception as e:
            logger.error(f"Failed to save calibration: {e}")

    def load_calibration(self) -> bool:
        """Load calibration data from JSON if present."""
        if not os.path.exists(self.config_path):
            return False

        try:
            with open(self.config_path, "r") as f:
                data = json.load(f)
            self.arena_width_cm = data.get("arena_width_cm", self.arena_width_cm)
            self.arena_height_cm = data.get("arena_height_cm", self.arena_height_cm)
            src = data.get("source_points")
            if src and len(src) == 4:
                return self.set_calibration_points(src)
        except Exception as e:
            logger.warning(f"Could not load calibration file: {e}")
        return False

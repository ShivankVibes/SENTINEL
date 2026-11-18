"""ArUco marker tracking module for rover 2D pose estimation (x, y, heading)."""

import math
import logging
from typing import Optional, Dict, Any, List
import cv2
import numpy as np
from sentinel.vision.calibrator import ArenaCalibrator

logger = logging.getLogger("sentinel.tracker")


class RoverPose:
    """Estimated pose of the rover."""

    def __init__(
        self,
        marker_id: int,
        pixel_center: tuple,
        floor_x_cm: float,
        floor_y_cm: float,
        heading_deg: float,
        corners_px: List[tuple],
    ):
        self.marker_id = marker_id
        self.pixel_center = pixel_center  # (u, v)
        self.floor_x_cm = floor_x_cm      # cm
        self.floor_y_cm = floor_y_cm      # cm
        self.heading_deg = heading_deg    # degrees [-180, 180], 0 deg = +X axis (Right)
        self.corners_px = corners_px

    def to_dict(self) -> Dict[str, Any]:
        return {
            "marker_id": self.marker_id,
            "pixel_center": self.pixel_center,
            "x_cm": round(self.floor_x_cm, 1),
            "y_cm": round(self.floor_y_cm, 1),
            "heading_deg": round(self.heading_deg, 1),
        }


class ArucoRoverTracker:
    """Detects and tracks the mobile rover using an ArUco fiducial marker."""

    def __init__(
        self,
        calibrator: ArenaCalibrator,
        target_marker_id: int = 0,
        dict_name: str = "DICT_4X4_50",
    ):
        self.calibrator = calibrator
        self.target_marker_id = target_marker_id
        self.dict_name = dict_name

        # Load dictionary
        dict_attr = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)

        # Setup parameters
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.parameters = cv2.aruco.DetectorParameters()
            # Corner refinement
            if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
                self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        else:
            self.parameters = None

        # OpenCV 4.7+ detector object
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        else:
            self.detector = None

    def detect(self, frame: np.ndarray) -> Optional[RoverPose]:
        """Detect the target rover marker in the frame and compute its calibrated pose."""
        if frame is None or frame.size == 0:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame

        corners = []
        ids = None

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.parameters
            )

        if ids is None or len(ids) == 0:
            return None

        flat_ids = ids.flatten()
        target_idx = None
        for i, m_id in enumerate(flat_ids):
            if m_id == self.target_marker_id:
                target_idx = i
                break

        if target_idx is None:
            # If target marker not found, check if any marker is detected
            target_idx = 0

        marker_corners = corners[target_idx][0]  # Shape: (4, 2)
        # Corners: 0: top-left, 1: top-right, 2: bottom-right, 3: bottom-left
        c0 = marker_corners[0]
        c1 = marker_corners[1]
        c2 = marker_corners[2]
        c3 = marker_corners[3]

        # Centroid in pixels
        center_u = float(np.mean(marker_corners[:, 0]))
        center_v = float(np.mean(marker_corners[:, 1]))

        # Forward orientation vector: from marker center to midpoint of top edge (c0 -> c1)
        top_mid_u = float((c0[0] + c1[0]) / 2.0)
        top_mid_v = float((c0[1] + c1[1]) / 2.0)

        # Map center and forward point into metric ground coordinates
        floor_center_x, floor_center_y = self.calibrator.pixel_to_floor(center_u, center_v)
        floor_top_x, floor_top_y = self.calibrator.pixel_to_floor(top_mid_u, top_mid_v)

        # Compute heading angle in arena ground frame
        # Standard convention: 0 deg = along +X axis (Right), 90 deg = along +Y axis (Down)
        dx = floor_top_x - floor_center_x
        dy = floor_top_y - floor_center_y
        heading_rad = math.atan2(dy, dx)
        heading_deg = math.degrees(heading_rad)

        corners_list = [(float(pt[0]), float(pt[1])) for pt in marker_corners]

        return RoverPose(
            marker_id=int(flat_ids[target_idx]),
            pixel_center=(center_u, center_v),
            floor_x_cm=floor_center_x,
            floor_y_cm=floor_center_y,
            heading_deg=heading_deg,
            corners_px=corners_list,
        )

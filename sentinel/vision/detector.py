"""Hazard and anomaly detection module for SENTINEL.

Provides modular detection pipelines:
- Classical Computer Vision (HSV segmentation & contour geometry)
- Extensible AI / Object Detection stub for neural network models
Clearly distinguishes basic CV algorithms from AI/ML models.
"""

from abc import ABC, abstractmethod
from typing import List, Tuple, Optional, Dict, Any
import cv2
import numpy as np
from sentinel.vision.calibrator import ArenaCalibrator


class HazardDetection:
    """Represents a detected anomaly or hazard zone."""

    def __init__(
        self,
        hazard_id: int,
        label: str,
        detection_type: str,
        confidence: float,
        pixel_center: Tuple[float, float],
        pixel_box: Tuple[int, int, int, int],  # (x, y, w, h)
        floor_x_cm: float,
        floor_y_cm: float,
        radius_cm: float = 12.0,
    ):
        self.hazard_id = hazard_id
        self.label = label
        self.detection_type = detection_type  # e.g., 'CV_CONTOUR' vs 'AI_NEURAL_NET'
        self.confidence = confidence
        self.pixel_center = pixel_center
        self.pixel_box = pixel_box
        self.floor_x_cm = floor_x_cm
        self.floor_y_cm = floor_y_cm
        self.radius_cm = radius_cm

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.hazard_id,
            "label": self.label,
            "detection_type": self.detection_type,
            "confidence": round(self.confidence, 2),
            "pixel_center": [round(c, 1) for c in self.pixel_center],
            "pixel_box": list(self.pixel_box),
            "x_cm": round(self.floor_x_cm, 1),
            "y_cm": round(self.floor_y_cm, 1),
            "radius_cm": round(self.radius_cm, 1),
        }


class BaseHazardDetector(ABC):
    """Abstract base class for modular hazard detection engines."""

    @abstractmethod
    def detect(self, frame: np.ndarray) -> List[HazardDetection]:
        """Process image frame and return list of detected anomalies."""
        pass


class ColorContourHazardDetector(BaseHazardDetector):
    """Classical Computer Vision detector for designated high-visibility safe mock hazards.

    Uses HSV color thresholding, morphological noise filtering, and contour geometry.
    Note: This is a classical computer-vision technique, NOT an AI deep learning model.
    """

    def __init__(
        self,
        calibrator: ArenaCalibrator,
        lower_hsv: Tuple[int, int, int] = (5, 120, 120),
        upper_hsv: Tuple[int, int, int] = (25, 255, 255),
        min_contour_area_px: int = 350,
        hazard_radius_cm: float = 12.0,
    ):
        self.calibrator = calibrator
        self.lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self.upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        self.min_contour_area_px = min_contour_area_px
        self.hazard_radius_cm = hazard_radius_cm
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def detect(self, frame: np.ndarray) -> List[HazardDetection]:
        if frame is None or frame.size == 0:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # Morphological open to remove noise, close to fill gaps
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for idx, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area_px:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            # Centroid via image moments
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = float(M["m10"] / M["m00"])
                cy = float(M["m01"] / M["m00"])
            else:
                cx, cy = float(x + w / 2.0), float(y + h / 2.0)

            # Map to physical floor coordinates
            floor_x, floor_y = self.calibrator.pixel_to_floor(cx, cy)

            # Heuristic confidence based on area compactness
            perimeter = cv2.arcLength(cnt, True)
            circularity = 4 * np.pi * (area / (perimeter * perimeter)) if perimeter > 0 else 0.5
            confidence = min(0.99, max(0.65, float(circularity)))

            detections.append(
                HazardDetection(
                    hazard_id=idx + 1,
                    label="MOCK_HAZARD_ZONE",
                    detection_type="CV_COLOR_CONTOUR",
                    confidence=confidence,
                    pixel_center=(cx, cy),
                    pixel_box=(x, y, w, h),
                    floor_x_cm=floor_x,
                    floor_y_cm=floor_y,
                    radius_cm=self.hazard_radius_cm,
                )
            )

        return detections


class AIHazardDetector(BaseHazardDetector):
    """Deep learning neural network detector stub for object classification/detection.

    Designed for lightweight models (e.g. YOLOv8n / MobileNet-SSD ONNX or CoreML).
    Distinguishes AI inference from standard color thresholding.
    """

    def __init__(self, calibrator: ArenaCalibrator, model_path: Optional[str] = None):
        self.calibrator = calibrator
        self.model_path = model_path
        self.is_model_loaded = False
        # Pluggable backend structure

    def detect(self, frame: np.ndarray) -> List[HazardDetection]:
        # Placeholder for trained AI model inference pipeline
        # Returns empty list when no neural weights are mounted
        return []

"""Visual annotation and HUD overlay generator for SENTINEL video feed."""

import math
from typing import Optional, List, Tuple
import cv2
import numpy as np
from sentinel.vision.calibrator import ArenaCalibrator
from sentinel.vision.tracker import RoverPose
from sentinel.vision.detector import HazardDetection


class FrameAnnotator:
    """Draws tactical telemetry, detection overlays, and HUD diagnostics directly onto frames."""

    def __init__(self, calibrator: ArenaCalibrator):
        self.calibrator = calibrator

    def render(
        self,
        frame: np.ndarray,
        rover_pose: Optional[RoverPose] = None,
        hazards: Optional[List[HazardDetection]] = None,
        path_cm: Optional[List[Tuple[float, float]]] = None,
        mode_str: str = "SIMULATION",
        status_text: str = "MONITORING",
        fps: float = 0.0,
    ) -> np.ndarray:
        if frame is None or frame.size == 0:
            return frame

        canvas = frame.copy()

        # 1. Draw calibrated arena boundaries
        if self.calibrator.source_points is not None:
            pts = self.calibrator.source_points.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(canvas, [pts], isClosed=True, color=(0, 220, 255), thickness=2)
            cv2.putText(
                canvas,
                f"ARENA: {int(self.calibrator.arena_width_cm)}x{int(self.calibrator.arena_height_cm)}cm",
                (pts[0][0][0] + 5, pts[0][0][1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 220, 255),
                1,
                cv2.LINE_AA,
            )

        # 2. Draw planned navigation path
        if path_cm and len(path_cm) > 1:
            px_path = [self.calibrator.floor_to_pixel(x, y) for (x, y) in path_cm]
            for i in range(len(px_path) - 1):
                cv2.line(canvas, px_path[i], px_path[i + 1], (0, 255, 120), 2, cv2.LINE_AA)
            for idx, pt in enumerate(px_path):
                cv2.circle(canvas, pt, 4, (0, 255, 180), -1)

        # 3. Draw detected hazards
        if hazards:
            for h in hazards:
                bx, by, bw, bh = h.pixel_box
                cx, cy = int(h.pixel_center[0]), int(h.pixel_center[1])
                # Hazard box
                cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), (0, 60, 255), 2)
                # Centroid crosshair
                cv2.drawMarker(canvas, (cx, cy), (0, 60, 255), cv2.MARKER_TILTED_CROSS, 14, 2)

                # Metric label
                label = f"HAZARD #{h.hazard_id} [{h.floor_x_cm}, {h.floor_y_cm}cm]"
                sub = f"{h.detection_type} ({int(h.confidence * 100)}%)"
                cv2.putText(
                    canvas, label, (bx, max(20, by - 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 100, 255), 1, cv2.LINE_AA
                )
                cv2.putText(
                    canvas, sub, (bx, max(35, by - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 160, 255), 1, cv2.LINE_AA
                )

        # 4. Draw Rover Pose
        if rover_pose is not None:
            # Draw marker boundary
            pts = np.array(rover_pose.corners_px, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(canvas, [pts], isClosed=True, color=(255, 100, 0), thickness=2)

            rcx, rcy = int(rover_pose.pixel_center[0]), int(rover_pose.pixel_center[1])
            cv2.circle(canvas, (rcx, rcy), 5, (255, 200, 0), -1)

            # Draw heading arrow
            arrow_len = 35
            heading_rad = math.radians(rover_pose.heading_deg)
            # In pixel frame, dx = cos, dy = sin
            arrow_end_x = int(rcx + arrow_len * math.cos(heading_rad))
            arrow_end_y = int(rcy + arrow_len * math.sin(heading_rad))
            cv2.arrowedLine(
                canvas, (rcx, rcy), (arrow_end_x, arrow_end_y), (0, 255, 255), 2, tipLength=0.3
            )

            # Coordinate badge
            rover_label = f"ROVER #{rover_pose.marker_id}: ({rover_pose.floor_x_cm}, {rover_pose.floor_y_cm}cm) {rover_pose.heading_deg}°"
            cv2.putText(
                canvas, rover_label, (rcx - 60, rcy + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA
            )

        # 5. Top Diagnostics Banner (HUD)
        h, w = canvas.shape[:2]
        hud_bar = np.zeros((36, w, 3), dtype=np.uint8)
        # Semi-transparent blend
        cv2.addWeighted(canvas[0:36, 0:w], 0.3, hud_bar, 0.7, 0, canvas[0:36, 0:w])

        mode_color = (0, 200, 255) if "SIM" in mode_str.upper() else (0, 255, 100)
        cv2.putText(
            canvas,
            f"SENTINEL SYSTEM [{mode_str.upper()}]",
            (14, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            mode_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"STATE: {status_text}  |  FPS: {fps:.1f}",
            (320, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )

        return canvas

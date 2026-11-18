#!/usr/bin/env python3
"""Comprehensive test for SENTINEL Computer Vision Pipeline (Calibration, Tracking, Detection, Overlay)."""

import cv2
import numpy as np
from sentinel.vision.calibrator import ArenaCalibrator
from sentinel.vision.tracker import ArucoRoverTracker
from sentinel.vision.detector import ColorContourHazardDetector
from sentinel.vision.overlay import FrameAnnotator

def test_full_vision_pipeline():
    print("Testing SENTINEL Full Vision Pipeline...")

    # 1. Setup Calibrator
    calibrator = ArenaCalibrator(arena_width_cm=200.0, arena_height_cm=150.0)
    # Define 4 calibration pixel corners
    corners = [(100, 50), (1180, 50), (1180, 670), (100, 670)]
    success = calibrator.set_calibration_points(corners)
    assert success, "Calibrator failed to compute homography."
    print("[OK] Homography calibration matrix computed.")

    # Test coordinate mapping
    u, v = 640, 360  # image center
    x_cm, y_cm = calibrator.pixel_to_floor(u, v)
    print(f"Image Center ({u}, {v}) -> Floor ({x_cm}, {y_cm}) cm")
    back_u, back_v = calibrator.floor_to_pixel(x_cm, y_cm)
    assert abs(back_u - u) <= 2 and abs(back_v - v) <= 2, f"Coordinate cycle drift: {back_u}, {back_v}"
    print("[OK] Bidirectional coordinate transformation verified.")

    # 2. Synthesize a test frame with an ArUco marker and an Orange Hazard patch
    frame = np.full((720, 1280, 3), 40, dtype=np.uint8)

    # Draw simulated mock hazard: Vibrant orange circle
    # Orange in BGR: (0, 140, 255)
    cv2.circle(frame, (850, 350), 35, (0, 140, 255), -1)

    # Generate and draw ArUco marker #0 onto frame at (300, 350)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, 0, 80)
    marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    frame[310:390, 260:340] = marker_bgr

    # 3. Test Tracker
    tracker = ArucoRoverTracker(calibrator=calibrator, target_marker_id=0)
    pose = tracker.detect(frame)
    assert pose is not None, "Failed to detect ArUco marker."
    print(f"[OK] Rover Pose Detected: ID={pose.marker_id}, Floor=({pose.floor_x_cm}, {pose.floor_y_cm})cm, Heading={pose.heading_deg}°")

    # 4. Test Hazard Detector
    detector = ColorContourHazardDetector(calibrator=calibrator)
    hazards = detector.detect(frame)
    assert len(hazards) > 0, "Failed to detect mock hazard patch."
    h0 = hazards[0]
    print(f"[OK] Hazard Detected: ID={h0.hazard_id}, Type={h0.detection_type}, Conf={h0.confidence}, Floor=({h0.floor_x_cm}, {h0.floor_y_cm})cm")

    # 5. Test Overlay Annotator
    annotator = FrameAnnotator(calibrator=calibrator)
    annotated = annotator.render(
        frame=frame,
        rover_pose=pose,
        hazards=hazards,
        path_cm=[(pose.floor_x_cm, pose.floor_y_cm), (h0.floor_x_cm, h0.floor_y_cm)],
        mode_str="SIMULATION",
        status_text="NAVIGATING_TO_TARGET",
        fps=30.0,
    )
    assert annotated.shape == (720, 1280, 3), "Annotated frame shape mismatch."
    print("[OK] Frame Annotator successfully rendered telemetry HUD.")
    print("\n[SUCCESS] Milestone 3 Computer Vision Pipeline Passed 100%.")

if __name__ == "__main__":
    test_full_vision_pipeline()

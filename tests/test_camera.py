#!/usr/bin/env python3
"""Test script for SENTINEL Camera Subsystem."""

import time
import sys
from sentinel.vision.camera import CameraManager

def test_camera_subsystem():
    print("Testing SENTINEL Camera Manager...")
    cam = CameraManager(device_index=0, width=1280, height=720, fps=30, synthetic_fallback=True)
    
    started = cam.start()
    assert started, "Camera manager failed to start."
    print(f"Initial Status: {cam.get_status()}")

    # Collect frames for 2 seconds
    time.sleep(1.0)
    ret, frame = cam.read()
    assert ret and frame is not None, "Failed to read frame from CameraManager."
    print(f"Captured Frame Shape: {frame.shape}, Dtype: {frame.dtype}")

    time.sleep(1.0)
    status = cam.get_status()
    print(f"Updated Status after 2s: {status}")
    assert status["frame_count"] > 10, "Frame counter did not advance."

    cam.stop()
    print("[SUCCESS] Camera Subsystem test completed cleanly.")

if __name__ == "__main__":
    test_camera_subsystem()

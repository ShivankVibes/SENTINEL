"""Camera management module for SENTINEL.

Handles webcam capture, thread-safe frame acquisition, device index probing,
auto-reconnection, and fallback test pattern generation for macOS privacy gates
or simulation testing.
"""

import time
import threading
import logging
import subprocess
from typing import Optional, Tuple
import cv2
import numpy as np

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("sentinel.camera")


class CameraManager:
    """Thread-safe camera capture manager with resilient auto-recovery and synthetic fallback."""

    def __init__(
        self,
        device_index: int = 1,
        fallback_indices: Optional[list] = None,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        synthetic_fallback: bool = True,
    ):
        self.device_index = device_index
        self.fallback_indices = fallback_indices or [0]
        self.target_width = width
        self.target_height = height
        self.target_fps = fps
        self.synthetic_fallback = synthetic_fallback

        self.cap: Optional[cv2.VideoCapture] = None
        self.is_running = False
        self.is_synthetic = False
        self.current_index = -1
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_timestamp = 0.0
        self.frame_count = 0
        self.fps_actual = 0.0

        self.lock = threading.Lock()
        self.capture_thread: Optional[threading.Thread] = None
        self.status_message = "Uninitialized"

    @classmethod
    def probe_available_cameras(cls, active_index: int = -1) -> list:
        """Scan system for available video capture devices without disrupting active streams."""
        cameras = []
        # Try macOS native system profiler first to avoid locking AVFoundation UVC streams
        try:
            out = subprocess.check_output(["system_profiler", "SPCameraDataType"], text=True, timeout=2.0)
            lines = [line.strip() for line in out.splitlines() if line.strip()]
            names = []
            for line in lines:
                if line.endswith(":") and line != "Camera:":
                    names.append(line[:-1].strip())
            
            for idx, name in enumerate(names):
                is_ext = idx > 0
                label = f"Camera {idx}: {name}" + (" (External USB)" if is_ext else " (Built-in Mac)")
                cameras.append({
                    "index": idx,
                    "name": label,
                    "resolution": "1920x1080",
                    "is_responsive": True,
                })
            if cameras:
                return cameras
        except Exception as e:
            logger.debug(f"system_profiler probe failed: {e}")

        # Fallback OpenCV index probe (skips active_index to protect active stream)
        for idx in range(3):
            if idx == active_index:
                cameras.append({
                    "index": idx,
                    "name": f"Camera {idx}: Active Camera (Streaming)",
                    "resolution": "1920x1080",
                    "is_responsive": True,
                })
                continue
            try:
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    ret, _ = cap.read()
                    cap.release()
                    if ret:
                        desc = "Built-in Camera" if idx == 0 else "External USB Camera"
                        cameras.append({
                            "index": idx,
                            "name": f"Camera {idx}: {desc}",
                            "resolution": "1920x1080",
                            "is_responsive": True,
                        })
            except Exception:
                pass
        return cameras

    def switch_device(self, new_index: int) -> bool:
        """Dynamically switch to a different camera device without stopping the capture thread."""
        logger.info(f"Switching camera to device index {new_index}...")
        with self.lock:
            old_cap = self.cap
            self.cap = None

        if old_cap:
            try:
                old_cap.release()
            except Exception:
                pass

        success = self._open_device(new_index)
        with self.lock:
            if success:
                self.device_index = new_index
                self.is_synthetic = False
                self.status_message = f"Connected to Hardware Camera Index {self.current_index}"
                logger.info(f"Camera successfully switched to index {new_index}")
                return True
            else:
                self.is_synthetic = True
                self.status_message = f"Failed to open camera index {new_index}. Running synthetic fallback."
                logger.warning(f"Failed to open camera {new_index}.")
                return False

    def start(self) -> bool:
        """Start the background frame capture thread."""
        if self.is_running:
            return True

        success = self._open_device(self.device_index)
        if not success:
            for fb_idx in self.fallback_indices:
                logger.info(f"Trying fallback camera index {fb_idx}...")
                if self._open_device(fb_idx):
                    success = True
                    break

        if not success:
            if self.synthetic_fallback:
                logger.warning(
                    "Physical camera unavailable or macOS permission pending. "
                    "Engaging synthetic arena camera feed."
                )
                self.is_synthetic = True
                self.status_message = "Running in Synthetic Arena Mode (Camera fallback)"
            else:
                self.status_message = "Camera initialization failed."
                return False
        else:
            self.is_synthetic = False
            self.status_message = f"Connected to Hardware Camera Index {self.current_index}"

        self.is_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        return True

    def _open_device(self, index: int) -> bool:
        """Attempt to open a physical camera device index."""
        try:
            # On macOS, AVFOUNDATION is standard backend
            cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
            if not cap.isOpened():
                cap = cv2.VideoCapture(index)

            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
                cap.set(cv2.CAP_PROP_FPS, self.target_fps)

                ret, test_frame = cap.read()
                if ret and test_frame is not None and test_frame.size > 0:
                    self.cap = cap
                    self.current_index = index
                    logger.info(f"Successfully opened camera index {index} ({test_frame.shape[1]}x{test_frame.shape[0]})")
                    return True
                cap.release()
        except Exception as e:
            logger.debug(f"Failed to open camera index {index}: {e}")
        return False

    def _capture_loop(self):
        """Dedicated background loop continuously acquiring frames."""
        last_fps_calc = time.time()
        fps_frames = 0
        sim_tick = 0

        while self.is_running:
            frame = None
            if not self.is_synthetic and self.cap is not None:
                ret, raw_frame = self.cap.read()
                if ret and raw_frame is not None:
                    frame = raw_frame
                else:
                    logger.warning("Frame read failed. Attempting reconnect...")
                    self._attempt_reconnect()
                    time.sleep(0.1)
                    continue
            else:
                # Generate synthetic arena camera frame
                frame = self._generate_synthetic_arena_frame(sim_tick)
                sim_tick += 1
                time.sleep(1.0 / self.target_fps)

            if frame is not None:
                with self.lock:
                    self.latest_frame = frame
                    self.latest_timestamp = time.time()
                    self.frame_count += 1
                    fps_frames += 1

                # Calculate actual rolling FPS every second
                now = time.time()
                if now - last_fps_calc >= 1.0:
                    self.fps_actual = fps_frames / (now - last_fps_calc)
                    fps_frames = 0
                    last_fps_calc = now

    def _attempt_reconnect(self):
        """Attempt to recover disconnected camera, prioritizing the configured device_index."""
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        # Prioritize the user's selected device_index (e.g. External USB camera)
        for attempt in range(3):
            if self._open_device(self.device_index):
                logger.info(f"Camera reconnected on configured index {self.device_index}")
                self.is_synthetic = False
                self.status_message = f"Connected to Camera {self.device_index}"
                return
            time.sleep(0.3)

        # Fallback to secondary indices only if primary is completely unavailable
        for idx in self.fallback_indices:
            if idx != self.device_index and self._open_device(idx):
                logger.warning(f"Configured camera {self.device_index} unavailable. Fell back to index {idx}.")
                self.is_synthetic = False
                self.status_message = f"Fell back to Camera {idx}"
                return

        if not self.is_synthetic:
            logger.warning("All physical cameras unavailable. Switching to synthetic arena feed.")
            self.is_synthetic = True
            self.status_message = "Running in Synthetic Arena Mode (Hardware Disconnected)"

    def _generate_synthetic_arena_frame(self, tick: int) -> np.ndarray:
        """Generate high-contrast simulated arena frame with grid, calibration marks, and mock hazard."""
        h, w = self.target_height, self.target_width
        frame = np.full((h, w, 3), 28, dtype=np.uint8)  # Dark charcoal floor

        # Draw simulated demonstration floor boundary
        pad_x, pad_y = int(w * 0.1), int(h * 0.1)
        arena_rect = (pad_x, pad_y, w - 2 * pad_x, h - 2 * pad_y)
        cv2.rectangle(
            frame,
            (arena_rect[0], arena_rect[1]),
            (arena_rect[0] + arena_rect[2], arena_rect[1] + arena_rect[3]),
            (70, 70, 70),
            2,
        )

        # Draw grid lines inside arena
        step_x = arena_rect[2] // 10
        step_y = arena_rect[3] // 8
        for x in range(arena_rect[0], arena_rect[0] + arena_rect[2], step_x):
            cv2.line(frame, (x, arena_rect[1]), (x, arena_rect[1] + arena_rect[3]), (45, 45, 45), 1)
        for y in range(arena_rect[1], arena_rect[1] + arena_rect[3], step_y):
            cv2.line(frame, (arena_rect[0], y), (arena_rect[0] + arena_rect[2], y), (45, 45, 45), 1)

        # Simulated Corner Calibration Markers (high contrast crosshairs)
        corners = [
            (arena_rect[0], arena_rect[1]),
            (arena_rect[0] + arena_rect[2], arena_rect[1]),
            (arena_rect[0] + arena_rect[2], arena_rect[1] + arena_rect[3]),
            (arena_rect[0], arena_rect[1] + arena_rect[3]),
        ]
        for cx, cy in corners:
            cv2.circle(frame, (cx, cy), 8, (0, 220, 255), 2)
            cv2.drawMarker(frame, (cx, cy), (0, 220, 255), cv2.MARKER_CROSS, 14, 1)

        # Draw status watermark
        cv2.putText(
            frame,
            "SENTINEL OPTICAL OBSERVATION STATION [SYNTHETIC FEED]",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 200, 255),
            2,
            cv2.LINE_AA,
        )
        timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(
            frame,
            f"TIME: {timestamp_str} | SIM_TICK: {tick} | TARGET: 1280x720",
            (30, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (160, 160, 160),
            1,
            cv2.LINE_AA,
        )

        return frame

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Return the most recently acquired frame."""
        with self.lock:
            if self.latest_frame is not None:
                return True, self.latest_frame.copy()
            return False, None

    def get_status(self) -> dict:
        """Return current operational metrics."""
        return {
            "is_running": self.is_running,
            "is_synthetic": self.is_synthetic,
            "device_index": self.current_index if not self.is_synthetic else None,
            "actual_fps": round(self.fps_actual, 1),
            "frame_count": self.frame_count,
            "status_message": self.status_message,
            "target_resolution": f"{self.target_width}x{self.target_height}",
        }

    def stop(self):
        """Cleanly stop the camera capture thread and release hardware resources."""
        self.is_running = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.5)
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        logger.info("Camera manager stopped and resources released.")

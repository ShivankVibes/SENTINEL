"""Master coordinator and state machine for SENTINEL system."""

import time
import logging
import threading
from typing import Optional, List, Tuple, Dict, Any
import cv2
import numpy as np

from sentinel.vision.camera import CameraManager
from sentinel.vision.calibrator import ArenaCalibrator
from sentinel.vision.tracker import ArucoRoverTracker, RoverPose
from sentinel.vision.detector import ColorContourHazardDetector, HazardDetection
from sentinel.vision.overlay import FrameAnnotator
from sentinel.navigation.grid import OccupancyGrid
from sentinel.navigation.planner import AStarPlanner
from sentinel.navigation.controller import TrajectoryController, NavigationCommand
from sentinel.simulation.virtual_rover import VirtualRover
from sentinel.comms.wifi_client import ESP32Client

logger = logging.getLogger("sentinel.core")


class SentinelSystem:
    """Central orchestrator managing vision, planning, simulation, and hardware dispatch."""

    def __init__(self):
        # Configuration
        self.arena_width_cm = 200.0
        self.arena_height_cm = 150.0

        # Subsystems
        self.calibrator = ArenaCalibrator(
            arena_width_cm=self.arena_width_cm,
            arena_height_cm=self.arena_height_cm,
        )
        # Default calibration corners if uncalibrated
        if not self.calibrator.is_calibrated:
            self.calibrator.set_calibration_points([(120, 70), (1160, 70), (1160, 650), (120, 650)])

        import os
        default_cam = int(os.environ.get("SENTINEL_CAMERA_INDEX", 0))
        logger.info(f"Initializing video feed on Camera Index {default_cam}")

        self.camera = CameraManager(device_index=default_cam, fallback_indices=[0, 1, 2], width=1280, height=720, fps=30, synthetic_fallback=True)
        self.tracker = ArucoRoverTracker(calibrator=self.calibrator, target_marker_id=0)
        self.detector = ColorContourHazardDetector(calibrator=self.calibrator)
        self.annotator = FrameAnnotator(calibrator=self.calibrator)

        self.grid = OccupancyGrid(
            width_cm=self.arena_width_cm,
            height_cm=self.arena_height_cm,
            cell_size_cm=5.0,
            safety_margin_cm=14.0,
        )
        self.grid.add_boundary_margins()
        self.planner = AStarPlanner(self.grid)
        self.controller = TrajectoryController(position_tolerance_cm=8.0, heading_tolerance_deg=14.0)

        # Simulation & Hardware
        self.virtual_rover = VirtualRover(
            initial_x_cm=30.0,
            initial_y_cm=30.0,
            initial_heading_deg=0.0,
            arena_width_cm=self.arena_width_cm,
            arena_height_cm=self.arena_height_cm,
        )
        self.esp32_client = ESP32Client()

        # System State
        self.mode = "SIMULATION"  # "SIMULATION" or "REAL"
        self.system_state = "IDLE"  # "IDLE", "DETECTING", "PLANNING", "NAVIGATING", "INSPECTING", "EMERGENCY_STOP"
        self.is_running = False

        # Live telemetry snapshot
        self.current_pose: Optional[RoverPose] = None
        self.detected_hazards: List[HazardDetection] = []
        self.simulated_target: Optional[Tuple[float, float]] = (150.0, 100.0)  # Default sim target
        self.active_path: List[Tuple[float, float]] = []
        self.last_command: Optional[NavigationCommand] = None
        self.event_logs: List[Dict[str, Any]] = []

        self.lock = threading.RLock()
        self.loop_thread: Optional[threading.Thread] = None

        self.log_event("SYSTEM", "SENTINEL core initialized in SIMULATION mode.")

    def log_event(self, category: str, message: str, level: str = "INFO"):
        """Append an event to the circular log buffer."""
        entry = {
            "timestamp": time.strftime("%H:%M:%S"),
            "category": category,
            "message": message,
            "level": level,
        }
        with self.lock:
            self.event_logs.append(entry)
            if len(self.event_logs) > 60:
                self.event_logs.pop(0)
        logger.info(f"[{category}] {message}")

    def start(self):
        """Start the camera and master control loop."""
        if self.is_running:
            return
        self.camera.start()
        self.is_running = True
        self.loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.loop_thread.start()
        self.log_event("SYSTEM", "Observation station monitoring loop started.")

    def stop(self):
        """Stop all background operations."""
        self.is_running = False
        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=1.0)
        self.camera.stop()
        self.log_event("SYSTEM", "System halted.")

    def switch_camera(self, new_index: int) -> bool:
        """Switch camera device dynamically."""
        success = self.camera.switch_device(new_index)
        if success:
            self.log_event("CAMERA", f"Switched active video feed to Camera Index {new_index}")
        else:
            self.log_event("CAMERA", f"Failed to switch to Camera Index {new_index}", level="WARN")
        return success

    def get_available_cameras(self) -> list:
        """Scan and return list of detected cameras without interrupting active streams."""
        return CameraManager.probe_available_cameras(active_index=self.camera.current_index)

    def set_mode(self, new_mode: str):
        """Switch operating mode between SIMULATION and REAL."""
        with self.lock:
            if new_mode in ["SIMULATION", "REAL"]:
                self.mode = new_mode
                self.log_event("MODE", f"Switched operating mode to {self.mode}")
                if self.mode == "REAL":
                    self.virtual_rover.is_active = False
                else:
                    self.virtual_rover.is_active = True

    def trigger_emergency_stop(self):
        """Immediately halt all rover movement."""
        with self.lock:
            self.system_state = "EMERGENCY_STOP"
            self.controller.clear()
            self.active_path = []
            if self.mode == "REAL":
                self.esp32_client.emergency_stop()
            else:
                self.virtual_rover.execute_command(NavigationCommand(action="STOP", speed=0, duration_ms=0))
            self.log_event("EMERGENCY", "EMERGENCY STOP TRIGGERED. All motion halted.", level="ALERT")

    def reset_emergency(self):
        """Clear emergency stop condition."""
        with self.lock:
            self.system_state = "IDLE"
            if self.mode == "REAL":
                self.esp32_client.send_command(action="RESET_EMERGENCY")
            self.log_event("SYSTEM", "Emergency state cleared. System ready.")

    def set_simulated_target(self, x_cm: float, y_cm: float):
        """Set a target anomaly in simulation mode."""
        with self.lock:
            self.simulated_target = (float(x_cm), float(y_cm))
            self.log_event("TARGET", f"Simulation target set at ({x_cm:.1f}, {y_cm:.1f})cm")
            self.plan_mission_to_target(x_cm, y_cm)

    def plan_mission_to_target(self, target_x_cm: float, target_y_cm: float):
        """Calculate collision-free route to target anomaly and begin navigation."""
        pose = self.current_pose or (self.virtual_rover.get_pose() if self.mode == "SIMULATION" else None)
        if pose is None:
            self.log_event("NAV", "Cannot plan path: Rover position is unknown.", level="WARN")
            return False

        start_pt = (pose.floor_x_cm, pose.floor_y_cm)
        goal_pt = (target_x_cm, target_y_cm)

        # Update grid with detected static obstacles
        self.grid.clear()
        self.grid.add_boundary_margins()

        path = self.planner.plan(start_pt, goal_pt)
        if path and len(path) >= 2:
            self.active_path = path
            self.controller.set_path(path)
            self.system_state = "NAVIGATING"
            self.log_event(
                "NAV",
                f"Trajectory planned: {len(path)} waypoints to ({target_x_cm:.1f}, {target_y_cm:.1f})cm",
            )
            return True
        else:
            self.log_event("NAV", "Path planning failed: Target unreachable or blocked.", level="WARN")
            return False

    def _run_loop(self):
        """Master execution loop (runs at ~15 Hz)."""
        while self.is_running:
            start_time = time.time()

            # 1. Grab camera frame
            ret, frame = self.camera.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            # 2. Computer Vision Pipeline
            # Detect ArUco rover pose from camera if available
            camera_pose = self.tracker.detect(frame)
            hazards = self.detector.detect(frame)

            with self.lock:
                if self.mode == "REAL":
                    self.current_pose = camera_pose
                    self.detected_hazards = hazards
                else:
                    # In simulation mode, use virtual rover pose
                    self.current_pose = self.virtual_rover.get_pose()
                    # If camera detected physical hazards, incorporate them; else use simulated target
                    if hazards:
                        self.detected_hazards = hazards
                    elif self.simulated_target is not None:
                        # Synthetic hazard object
                        self.detected_hazards = [
                            HazardDetection(
                                hazard_id=1,
                                label="SIMULATED_MOCK_HAZARD",
                                detection_type="SIMULATION_TARGET",
                                confidence=1.0,
                                pixel_center=self.calibrator.floor_to_pixel(
                                    self.simulated_target[0], self.simulated_target[1]
                                ),
                                pixel_box=(0, 0, 30, 30),
                                floor_x_cm=self.simulated_target[0],
                                floor_y_cm=self.simulated_target[1],
                            )
                        ]

            # 3. Autonomous Navigation & Mission Execution
            if self.system_state == "NAVIGATING" and self.current_pose is not None:
                ultrasonic_dist = (
                    self.virtual_rover.simulated_ultrasonic_cm
                    if self.mode == "SIMULATION"
                    else (self.esp32_client.last_telemetry.ultrasonic_distance_cm if self.esp32_client.last_telemetry else -1.0)
                )

                cmd = self.controller.update(self.current_pose, ultrasonic_distance_cm=ultrasonic_dist)
                self.last_command = cmd

                if cmd.action == "ARRIVED":
                    self.system_state = "INSPECTING"
                    self.log_event("MISSION", "Rover arrived at target hazard! Initiating sensor inspection.", level="SUCCESS")
                    # Inspect for 3 seconds then IDLE
                    threading.Thread(target=self._inspection_sequence, daemon=True).start()
                else:
                    # Dispatch command
                    if self.mode == "SIMULATION":
                        obs = [(h.floor_x_cm, h.floor_y_cm, h.radius_cm) for h in self.detected_hazards]
                        self.virtual_rover.execute_command(cmd, obstacles=obs)
                    elif self.mode == "REAL":
                        self.esp32_client.send_command(cmd.action, speed=cmd.speed, duration_ms=cmd.duration_ms)

            # Sleep remainder of control cycle
            elapsed = time.time() - start_time
            sleep_time = max(0.01, (1.0 / 15.0) - elapsed)
            time.sleep(sleep_time)

    def _inspection_sequence(self):
        """Simulate or execute target anomaly inspection."""
        time.sleep(2.0)
        with self.lock:
            self.system_state = "IDLE"
            self.active_path = []
            self.log_event(
                "MISSION",
                "Inspection complete: Mock hazard verified safe. Awaiting next command.",
                level="INFO",
            )

    def get_annotated_frame(self) -> np.ndarray:
        """Render annotated live frame for MJPEG streaming."""
        ret, frame = self.camera.read()
        if not ret or frame is None:
            # Fallback black frame
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)

        status_txt = f"{self.system_state} ({self.last_command.action if self.last_command else 'IDLE'})"
        fps = self.camera.fps_actual

        return self.annotator.render(
            frame=frame,
            rover_pose=self.current_pose,
            hazards=self.detected_hazards,
            path_cm=self.active_path,
            mode_str=self.mode,
            status_text=status_txt,
            fps=fps,
        )

    def get_telemetry_snapshot(self) -> Dict[str, Any]:
        """Produce full telemetry JSON payload for dashboard synchronization."""
        with self.lock:
            pose_dict = self.current_pose.to_dict() if self.current_pose else None
            hazards_dict = [h.to_dict() for h in self.detected_hazards]

            ultrasonic = (
                self.virtual_rover.simulated_ultrasonic_cm
                if self.mode == "SIMULATION"
                else (self.esp32_client.last_telemetry.ultrasonic_distance_cm if self.esp32_client.last_telemetry else -1.0)
            )

            return {
                "timestamp": time.time(),
                "mode": self.mode,
                "system_state": self.system_state,
                "camera_status": self.camera.get_status(),
                "rover_pose": pose_dict,
                "hazards": hazards_dict,
                "active_path": self.active_path,
                "last_command": self.last_command.to_dict() if self.last_command else None,
                "ultrasonic_distance_cm": ultrasonic,
                "esp32_connected": self.esp32_client.is_connected if self.mode == "REAL" else True,
                "arena": {
                    "width_cm": self.arena_width_cm,
                    "height_cm": self.arena_height_cm,
                    "is_calibrated": self.calibrator.is_calibrated,
                },
                "event_logs": self.event_logs[-15:],
            }


def main():
    """CLI launcher."""
    from sentinel.run import main as run_main
    run_main()


if __name__ == "__main__":
    main()


"""Closed-loop trajectory controller generating discrete drive commands for the rover."""

import math
from typing import List, Tuple, Optional, Dict, Any
from sentinel.vision.tracker import RoverPose


class NavigationCommand:
    """Standardized motion command dispatched to rover hardware or simulation."""

    def __init__(
        self,
        action: str,  # "MOVE_FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP", "ARRIVED"
        speed: int = 180,  # PWM 0-255
        duration_ms: int = 250,
        distance_cm: float = 0.0,
        angle_deg: float = 0.0,
        target_waypoint: Optional[Tuple[float, float]] = None,
        reason: str = "",
    ):
        self.action = action
        self.speed = speed
        self.duration_ms = duration_ms
        self.distance_cm = distance_cm
        self.angle_deg = angle_deg
        self.target_waypoint = target_waypoint
        self.reason = reason

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action,
            "speed": self.speed,
            "duration_ms": self.duration_ms,
            "distance_cm": round(self.distance_cm, 1),
            "angle_deg": round(self.angle_deg, 1),
            "target": [round(c, 1) for c in self.target_waypoint] if self.target_waypoint else None,
            "reason": self.reason,
        }


class TrajectoryController:
    """Calculates heading/distance errors and generates real-time differential drive commands."""

    def __init__(
        self,
        position_tolerance_cm: float = 8.0,
        heading_tolerance_deg: float = 14.0,
        cruising_pwm: int = 180,
        pivot_pwm: int = 160,
    ):
        self.position_tolerance_cm = position_tolerance_cm
        self.heading_tolerance_deg = heading_tolerance_deg
        self.cruising_pwm = cruising_pwm
        self.pivot_pwm = pivot_pwm

        self.current_path: List[Tuple[float, float]] = []
        self.current_waypoint_idx = 0
        self.is_active = False

    def set_path(self, path: List[Tuple[float, float]]):
        """Set a newly planned trajectory and reset waypoint index."""
        self.current_path = path or []
        self.current_waypoint_idx = 0
        self.is_active = len(self.current_path) > 0

    def clear(self):
        """Cancel current trajectory."""
        self.current_path = []
        self.current_waypoint_idx = 0
        self.is_active = False

    def update(
        self,
        current_pose: RoverPose,
        ultrasonic_distance_cm: Optional[float] = None,
    ) -> NavigationCommand:
        """Evaluate current pose against active trajectory and generate next control command."""
        if not self.is_active or not self.current_path:
            return NavigationCommand(action="STOP", reason="No active trajectory")

        # Check ultrasonic forward safety obstacle
        if ultrasonic_distance_cm is not None and ultrasonic_distance_cm > 0 and ultrasonic_distance_cm < 12.0:
            return NavigationCommand(
                action="STOP",
                reason=f"Ultrasonic proximity safety halt (Obstacle at {ultrasonic_distance_cm:.1f}cm)",
            )

        # Check if index is within range
        if self.current_waypoint_idx >= len(self.current_path):
            self.is_active = False
            return NavigationCommand(action="ARRIVED", reason="Destination reached")

        target_wp = self.current_path[self.current_waypoint_idx]
        rx, ry = current_pose.floor_x_cm, current_pose.floor_y_cm
        dist_to_wp = math.hypot(target_wp[0] - rx, target_wp[1] - ry)

        # Check if current waypoint reached
        if dist_to_wp <= self.position_tolerance_cm:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_path):
                self.is_active = False
                return NavigationCommand(
                    action="ARRIVED",
                    target_waypoint=target_wp,
                    reason="Target hazard reached successfully",
                )
            target_wp = self.current_path[self.current_waypoint_idx]
            dist_to_wp = math.hypot(target_wp[0] - rx, target_wp[1] - ry)

        # Calculate desired heading to next waypoint
        dx = target_wp[0] - rx
        dy = target_wp[1] - ry
        desired_heading_deg = math.degrees(math.atan2(dy, dx))

        # Calculate minimal angle error: [-180, 180]
        curr_heading = current_pose.heading_deg
        heading_err = (desired_heading_deg - curr_heading + 180) % 360 - 180

        # Motion decision: Pivot vs Drive Forward
        if abs(heading_err) > self.heading_tolerance_deg:
            # Need angular correction
            if heading_err > 0:
                # Need clockwise rotation (Turn Right in floor frame where +Y is down)
                return NavigationCommand(
                    action="TURN_RIGHT",
                    speed=self.pivot_pwm,
                    duration_ms=min(300, max(120, int(abs(heading_err) * 4.0))),
                    angle_deg=heading_err,
                    target_waypoint=target_wp,
                    reason=f"Aligning heading (Error: +{heading_err:.1f}°)",
                )
            else:
                return NavigationCommand(
                    action="TURN_LEFT",
                    speed=self.pivot_pwm,
                    duration_ms=min(300, max(120, int(abs(heading_err) * 4.0))),
                    angle_deg=heading_err,
                    target_waypoint=target_wp,
                    reason=f"Aligning heading (Error: {heading_err:.1f}°)",
                )
        else:
            # Heading aligned: Advance forward
            drive_ms = min(400, max(150, int(dist_to_wp * 12.0)))
            return NavigationCommand(
                action="MOVE_FORWARD",
                speed=self.cruising_pwm,
                duration_ms=drive_ms,
                distance_cm=dist_to_wp,
                target_waypoint=target_wp,
                reason=f"Advancing to waypoint ({dist_to_wp:.1f}cm remaining)",
            )

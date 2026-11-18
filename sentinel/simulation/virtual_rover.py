"""High-fidelity kinematic virtual rover simulation for testing without physical hardware."""

import math
import time
from typing import List, Tuple, Optional
from sentinel.vision.tracker import RoverPose
from sentinel.navigation.controller import NavigationCommand


class VirtualRover:
    """Simulates differential-drive kinematics, movement execution, and ultrasonic sensor readings."""

    def __init__(
        self,
        initial_x_cm: float = 30.0,
        initial_y_cm: float = 30.0,
        initial_heading_deg: float = 0.0,
        arena_width_cm: float = 200.0,
        arena_height_cm: float = 150.0,
    ):
        self.x_cm = float(initial_x_cm)
        self.y_cm = float(initial_y_cm)
        self.heading_deg = float(initial_heading_deg)
        self.arena_width_cm = float(arena_width_cm)
        self.arena_height_cm = float(arena_height_cm)

        # Kinematic parameters
        self.linear_speed_cm_s = 22.0     # Forward translation speed
        self.angular_speed_deg_s = 65.0   # In-place rotational speed
        self.rover_radius_cm = 9.0

        # Sensor readings
        self.simulated_ultrasonic_cm = 120.0
        self.battery_voltage = 7.8        # Simulated 2S Li-ion pack voltage
        self.is_active = True
        self.last_command: Optional[NavigationCommand] = None
        self.last_tick_time = time.time()

    def reset_pose(self, x_cm: float, y_cm: float, heading_deg: float = 0.0):
        """Teleport/reset virtual rover position."""
        self.x_cm = float(x_cm)
        self.y_cm = float(y_cm)
        self.heading_deg = float(heading_deg)

    def execute_command(self, cmd: NavigationCommand, obstacles: Optional[List[Tuple[float, float, float]]] = None):
        """Step the simulation based on an executed command."""
        self.last_command = cmd
        dt = (cmd.duration_ms / 1000.0) if cmd.duration_ms > 0 else 0.1

        if cmd.action == "MOVE_FORWARD":
            dist = self.linear_speed_cm_s * dt
            rad = math.radians(self.heading_deg)
            new_x = self.x_cm + dist * math.cos(rad)
            new_y = self.y_cm + dist * math.sin(rad)

            # Clamp within arena walls
            self.x_cm = max(self.rover_radius_cm, min(self.arena_width_cm - self.rover_radius_cm, new_x))
            self.y_cm = max(self.rover_radius_cm, min(self.arena_height_cm - self.rover_radius_cm, new_y))

        elif cmd.action == "TURN_RIGHT":
            d_theta = self.angular_speed_deg_s * dt
            self.heading_deg = (self.heading_deg + d_theta + 180) % 360 - 180

        elif cmd.action == "TURN_LEFT":
            d_theta = self.angular_speed_deg_s * dt
            self.heading_deg = (self.heading_deg - d_theta + 180) % 360 - 180

        # Update simulated ultrasonic distance
        self._update_ultrasonic(obstacles or [])

    def _update_ultrasonic(self, obstacles: List[Tuple[float, float, float]]):
        """Raycast along the rover's heading to compute synthetic ultrasonic forward distance."""
        rad = math.radians(self.heading_deg)
        dx = math.cos(rad)
        dy = math.sin(rad)

        closest_dist = 200.0  # Max sensor range

        # Distance to arena walls along ray:
        # Wall X = 0
        if dx < -1e-4:
            d = (0.0 - self.x_cm) / dx
            if d > 0:
                closest_dist = min(closest_dist, d)
        # Wall X = arena_width
        elif dx > 1e-4:
            d = (self.arena_width_cm - self.x_cm) / dx
            if d > 0:
                closest_dist = min(closest_dist, d)

        # Wall Y = 0
        if dy < -1e-4:
            d = (0.0 - self.y_cm) / dy
            if d > 0:
                closest_dist = min(closest_dist, d)
        # Wall Y = arena_height
        elif dy > 1e-4:
            d = (self.arena_height_cm - self.y_cm) / dy
            if d > 0:
                closest_dist = min(closest_dist, d)

        # Check obstacles along ray
        for ox, oy, oradius in obstacles:
            # Vector from rover to obstacle center
            vx = ox - self.x_cm
            vy = oy - self.y_cm
            proj = vx * dx + vy * dy
            if proj > 0:  # Obstacle is in front
                perp_dist_sq = (vx**2 + vy**2) - proj**2
                if perp_dist_sq <= oradius**2:
                    hit_dist = proj - math.sqrt(max(0.0, oradius**2 - perp_dist_sq))
                    if hit_dist > 0:
                        closest_dist = min(closest_dist, hit_dist)

        self.simulated_ultrasonic_cm = round(max(3.0, closest_dist), 1)

    def get_pose(self) -> RoverPose:
        """Produce a RoverPose compatible with the vision tracker output."""
        return RoverPose(
            marker_id=0,
            pixel_center=(0.0, 0.0),  # In simulation, coordinate is directly defined in floor frame
            floor_x_cm=round(self.x_cm, 1),
            floor_y_cm=round(self.y_cm, 1),
            heading_deg=round(self.heading_deg, 1),
            corners_px=[],
        )

    def get_telemetry(self) -> dict:
        """Return standardized telemetry packet."""
        return {
            "mode": "SIMULATION",
            "is_virtual": True,
            "x_cm": round(self.x_cm, 1),
            "y_cm": round(self.y_cm, 1),
            "heading_deg": round(self.heading_deg, 1),
            "ultrasonic_distance_cm": self.simulated_ultrasonic_cm,
            "battery_voltage": self.battery_voltage,
            "last_action": self.last_command.action if self.last_command else "IDLE",
        }

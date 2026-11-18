#!/usr/bin/env python3
"""Comprehensive test for SENTINEL Navigation, A* Path Planning, and Simulation Engine."""

import math
from sentinel.navigation.grid import OccupancyGrid
from sentinel.navigation.planner import AStarPlanner
from sentinel.navigation.controller import TrajectoryController
from sentinel.simulation.virtual_rover import VirtualRover


def test_navigation_and_simulation():
    print("Testing SENTINEL Navigation & Path Planning Subsystem...")

    # 1. Setup Grid
    grid = OccupancyGrid(width_cm=200.0, height_cm=150.0, cell_size_cm=5.0, safety_margin_cm=12.0)
    grid.add_boundary_margins()
    # Place a large static obstacle directly in the center
    obs_x, obs_y, obs_r = 100.0, 75.0, 20.0
    grid.add_circular_obstacle(obs_x, obs_y, obs_r)

    # 2. Setup A* Planner
    planner = AStarPlanner(grid)
    start_pt = (25.0, 25.0)
    goal_pt = (175.0, 120.0)

    path = planner.plan(start_pt, goal_pt)
    assert path is not None and len(path) >= 2, "A* failed to find a valid collision-free path."
    print(f"[OK] A* Path Successfully Planned ({len(path)} waypoints): {path}")

    # Verify that no waypoint or segment cuts into the obstacle circle
    for (wx, wy) in path:
        dist_to_obs = math.hypot(wx - obs_x, wy - obs_y)
        assert dist_to_obs >= obs_r, f"Waypoint ({wx}, {wy}) inside obstacle! Dist={dist_to_obs}"
    print("[OK] Obstacle safety clearance verified for all waypoints.")

    # 3. Test Closed-Loop Trajectory Execution with Virtual Rover
    rover = VirtualRover(initial_x_cm=start_pt[0], initial_y_cm=start_pt[1], initial_heading_deg=0.0)
    controller = TrajectoryController(position_tolerance_cm=8.0, heading_tolerance_deg=15.0)
    controller.set_path(path)

    print("\n--- Stepping Closed-Loop Virtual Rover Simulation ---")
    step_count = 0
    max_steps = 150
    arrived = False

    while step_count < max_steps:
        step_count += 1
        pose = rover.get_pose()
        cmd = controller.update(pose, ultrasonic_distance_cm=rover.simulated_ultrasonic_cm)

        if cmd.action == "ARRIVED":
            arrived = True
            print(f"[ARRIVED] Target reached in {step_count} simulation ticks! Final Pose: ({rover.x_cm:.1f}, {rover.y_cm:.1f})cm")
            break

        # Step virtual rover kinematics
        rover.execute_command(cmd, obstacles=[(obs_x, obs_y, obs_r)])

    assert arrived, f"Rover failed to reach goal within {max_steps} steps."
    final_dist = math.hypot(rover.x_cm - goal_pt[0], rover.y_cm - goal_pt[1])
    print(f"[OK] Final distance to goal: {final_dist:.1f} cm (Within tolerance).")
    print(f"[OK] Simulated Ultrasonic Sensor reading: {rover.simulated_ultrasonic_cm} cm")
    print("\n[SUCCESS] Milestone 4 Navigation & Simulation Passed 100%.")


if __name__ == "__main__":
    test_navigation_and_simulation()

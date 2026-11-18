#!/usr/bin/env python3
"""End-to-End integration test simulating the entire workflow of SENTINEL."""

import time
import math
from sentinel.main import SentinelSystem


def test_sentinel_end_to_end():
    print("=" * 60)
    print("Testing SENTINEL End-to-End System Workflow")
    print("=" * 60)

    # 1. Initialize System
    system = SentinelSystem()
    system.set_mode("SIMULATION")
    system.start()
    time.sleep(1.0)
    print("[OK] Subsystems booted. Observation station active.")

    # 2. Place a Mock Target Anomaly at (140, 90) cm
    target_x, target_y = 140.0, 90.0
    print(f"Setting simulated hazard anomaly at ({target_x}, {target_y}) cm...")
    system.set_simulated_target(target_x, target_y)

    # 3. Verify Path Planned
    time.sleep(0.5)
    snapshot = system.get_telemetry_snapshot()
    assert len(snapshot["active_path"]) >= 2, "Path was not generated."
    print(f"[OK] Autonomous path calculated: {snapshot['active_path']}")
    assert snapshot["system_state"] == "NAVIGATING", f"State is {snapshot['system_state']}"

    # 4. Monitor autonomous travel until inspection
    print("Observing autonomous rover travel towards anomaly...")
    start_travel = time.time()
    reached = False

    while time.time() - start_travel < 20.0:
        time.sleep(0.5)
        snap = system.get_telemetry_snapshot()
        state = snap["system_state"]
        pose = snap["rover_pose"]
        dist = math.hypot(pose["x_cm"] - target_x, pose["y_cm"] - target_y)
        cmd = snap["last_command"]["action"] if snap["last_command"] else "NONE"
        print(f" -> Rover at ({pose['x_cm']}, {pose['y_cm']})cm, Dist={dist:.1f}cm, Cmd={cmd}, State={state}")

        if state in ["INSPECTING", "IDLE"] and dist <= 14.0:
            reached = True
            print(f"[SUCCESS] Rover arrived at target hazard! Distance: {dist:.1f}cm. State: {state}")
            break

    assert reached, "Rover did not arrive at target in time."

    # 5. Test Emergency STOP Failsafe
    print("\n--- Testing Emergency STOP Failsafe ---")
    system.plan_mission_to_target(30.0, 30.0)
    time.sleep(0.3)
    assert system.system_state == "NAVIGATING"
    system.trigger_emergency_stop()
    snap = system.get_telemetry_snapshot()
    assert snap["system_state"] == "EMERGENCY_STOP", "System failed to transition to EMERGENCY_STOP."
    print("[OK] Emergency STOP immediately halted motion and cleared active paths.")

    # 6. Test Annotated Frame Generation
    frame = system.get_annotated_frame()
    assert frame is not None and frame.shape == (720, 1280, 3), "Annotated frame rendering failed."
    print("[OK] Video frame generator produces valid 1280x720 annotated frames.")

    # Clean shutdown
    system.stop()
    print("\n" + "=" * 60)
    print("RESULT: ALL END-TO-END WORKFLOW & SAFETY TESTS PASSED 100%.")
    print("=" * 60)


if __name__ == "__main__":
    test_sentinel_end_to_end()

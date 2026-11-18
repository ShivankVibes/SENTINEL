# SENTINEL — Autonomous AI Hazard Detection & Response System

**SENTINEL** is an AI-assisted optical observation station and autonomous ground rover navigation system engineered for real-time hazard detection, spatial coordinate estimation, trajectory planning, and autonomous inspection.

---

## 1. System Architecture

```
+-----------------------------------------------------------------------------------+
|                            SENTINEL ARCHITECTURE                                  |
+-----------------------------------------------------------------------------------+
|  [USB Observation Camera]                                                         |
|         │                                                                         |
|         ▼                                                                         |
|  [Vision & Localization Engine]                                                   |
|   • Camera auto-connect & fallback pattern generator                              |
|   • 4-Point Homography (Camera pixels ⟷ Metric floor coordinates in cm)           |
|   • ArUco Fiducial Marker Tracker (Rover [x, y, θ] heading estimation)            |
|   • Pluggable Hazard Detector (Classical CV color-contour / Neural net stub)      |
|         │                                                                         |
|         ▼                                                                         |
|  [Navigation & Planning Engine]                                                   |
|   • 2D Occupancy Grid with boundary inflation & obstacle safety margins           |
|   • A* Optimal Path Planning with line-of-sight waypoint smoothing                |
|   • Closed-Loop Trajectory Guidance (discrete differential steering commands)     |
|         │                                                                         |
|         ├─────────────────────────────────────────┐                               |
|         ▼                                         ▼                               |
|  [Simulation Subsystem]               [Hardware Communication Layer]              |
|   • Differential-drive kinematics      • Bi-directional JSON packet protocol      |
|   • Raycast forward ultrasonic range   • Wi-Fi REST/WebSocket command client      |
|   • Realistic motion physics ticks     • 500ms safety watchdog auto-shutdown      |
|         │                                         │                               |
|         └────────────────────┬────────────────────┘                               |
|                              ▼                                                    |
|                   [FastAPI Mission Server]                                        |
|                              │                                                    |
|           ┌──────────────────┴──────────────────┐                                 |
|           ▼                                     ▼                                 |
|  [Annotated MJPEG Video Feed]          [10Hz WebSocket Telemetry]                 |
|           │                                     │                                 |
|           └──────────────────┬──────────────────┘                                 |
|                              ▼                                                    |
|             [SENTINEL Mission Control Dashboard]                                  |
|   • Dark-mode tactical interface with responsive overhead canvas map             |
|   • Live observation stream with augmented reality overlays                      |
|   • Real-time ultrasonic range meter & telemetry gauges                           |
|   • Prominent Emergency STOP & Simulation/Real Mode toggle                        |
+-----------------------------------------------------------------------------------+
```

---

## 2. Quick Start

### A. Environment Setup
The system uses an isolated virtual environment (`.venv`) with Python 3.12:

```bash
cd /Users/shivankvibes/SENTINEL
source .venv/bin/activate
```

### B. Launch Mission Control
Start the unified FastAPI server, vision capture engine, and web dashboard:

```bash
python -m sentinel.run
```

Once running, open your browser to:
- **Mission Control Dashboard**: `http://localhost:8000`
- **Annotated Camera Stream**: `http://localhost:8000/video_feed`
- **Telemetry WebSocket**: `ws://localhost:8000/ws`

---

## 3. Operating Modes

### 1. Simulation Mode (Default)
Since physical rover assembly is ongoing, simulation mode allows full end-to-end testing:
- **Interactive Map**: Click anywhere on the tactical arena map in the dashboard to place a mock hazard anomaly.
- **Path Calculation**: The A* planner immediately calculates an obstacle-cleared route.
- **Rover Dispatch**: The virtual kinematic rover pivots, aligns its heading, accelerates along the trajectory, checks forward ultrasonic clearance, arrives at the target, and performs an inspection sequence.

### 2. Real Hardware Mode
Switching to **Real Hardware Mode** via the dashboard toggle switches tracking and control to the physical domain:
- **Rover Localization**: The observation camera tracks an **ArUco Marker** (Dictionary: `DICT_4X4_50`, Marker ID: `#0`) mounted flat on the rover.
- **Hardware Commands**: The communication client dispatches HTTP/WebSocket movement packets (`MOVE_FORWARD`, `TURN_LEFT`, `TURN_RIGHT`, `STOP`) to the ESP32 rover over local Wi-Fi.
- **Zero Fake Data**: In Real Mode, only live camera tracking and authenticated ESP32 sensor telemetry are rendered.

---

## 4. Hardware Verification & Wiring Guide

### Components:
- **Controller**: ESP32-12E
- **Motors**: 2 × TT Geared DC Motors + Wheels
- **Driver Board**: HW-094
- **Sensor**: Ultrasonic Sensor (e.g. HC-SR04 / HC-SR04P)
- **Power**: 2S Li-ion battery pack (7.4V) with common ground

> [!CAUTION]
> **Safety Rule**: Do NOT drive motors or wire the ESP32 until the HW-094 pinout is confirmed.

1. **Verify HW-094 Driver**:
   - Inspect the IC on the HW-094 board (commonly an L298N mini or MX1508 dual H-bridge).
   - Confirm motor power input terminals ($V_{CC}$, $GND$) and motor outputs ($OUT1/OUT2$, $OUT3/OUT4$).
   - Check input logic voltage (3.3V compatible).
2. **ESP32-12E Logic**:
   - Connect logic inputs to non-strapping pins (safe defaults in `firmware/esp32_sentinel/config.h`: GPIO 16, 17, 18, 19).
   - Ensure common ground between the battery, motor driver, and ESP32.
3. **Ultrasonic Sensor 3.3V Logic Protection**:
   - If using a 5V HC-SR04, the `ECHO` pin outputs 5V. Use a voltage divider (e.g., $1k\Omega$ / $2k\Omega$) to step down the signal to 3.3V before connecting to the ESP32 `ECHO` pin.
4. **Safety Watchdog**:
   - The ESP32 firmware includes a 500ms safety watchdog timer. If communication drops, the rover cuts motor power automatically.

---

## 5. Verification Test Suite

Run individual unit and integration tests from the project root:

```bash
# Verify environment and dependencies
.venv/bin/python tests/check_env.py

# Verify camera capture and frame rate
.venv/bin/python tests/test_camera.py

# Verify CV homography calibration, ArUco tracking, and hazard detection
.venv/bin/python tests/test_vision_pipeline.py

# Verify A* path planning, safety margins, and differential drive controller
.venv/bin/python tests/test_navigation.py

# Verify full end-to-end workflow (Anomaly -> Plan -> Travel -> Inspect -> Stop)
.venv/bin/python tests/test_end_to_end.py
```

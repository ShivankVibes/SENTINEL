"""Documented communication protocol definitions for SENTINEL rover telemetry and control."""

import time
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field


class CommandPacket(BaseModel):
    """Standard motion command packet dispatched to ESP32."""

    seq: int = Field(..., description="Monotonically increasing sequence number")
    action: str = Field(..., description="Action: MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, STOP, EMERGENCY_STOP, PING")
    speed: int = Field(default=180, ge=0, le=255, description="Motor PWM speed (0-255)")
    duration_ms: int = Field(default=250, ge=0, le=5000, description="Pulse duration in milliseconds")
    timestamp: float = Field(default_factory=time.time)


class TelemetryPacket(BaseModel):
    """Telemetry report received from ESP32 rover."""

    seq: int
    rover_id: str = "SENTINEL-MK1"
    is_connected: bool = True
    ultrasonic_distance_cm: float = -1.0
    battery_voltage: float = 0.0
    current_state: str = "IDLE"  # IDLE, MOVING, EMERGENCY_STOP, ERROR
    rssi_dbm: int = -50
    timestamp: float = Field(default_factory=time.time)


class CommandAck(BaseModel):
    """Acknowledgement returned by rover after processing command."""

    ack_seq: int
    status: str  # "OK", "REJECTED", "ERROR"
    message: str = ""
    timestamp: float = Field(default_factory=time.time)

"""Wi-Fi communication client for ESP32 rover control and telemetry monitoring."""

import time
import logging
import threading
from typing import Optional, Callable
import requests
from sentinel.comms.protocol import CommandPacket, TelemetryPacket, CommandAck

logger = logging.getLogger("sentinel.comms")


class ESP32Client:
    """Manages Wi-Fi communication, dispatching commands and monitoring rover health."""

    def __init__(
        self,
        esp32_ip: str = "192.168.4.1",
        port: int = 8080,
        timeout_sec: float = 0.5,
    ):
        self.esp32_ip = esp32_ip
        self.port = port
        self.timeout_sec = timeout_sec
        self.base_url = f"http://{esp32_ip}:{port}"

        self.seq_counter = 0
        self.is_connected = False
        self.last_telemetry: Optional[TelemetryPacket] = None
        self.last_contact_time = 0.0

        self.lock = threading.Lock()
        self.heartbeat_thread: Optional[threading.Thread] = None
        self.is_running = False

    def send_command(self, action: str, speed: int = 180, duration_ms: int = 250) -> Optional[CommandAck]:
        """Send a single motion command to the ESP32 rover."""
        self.seq_counter += 1
        cmd = CommandPacket(
            seq=self.seq_counter,
            action=action,
            speed=speed,
            duration_ms=duration_ms,
        )

        try:
            resp = requests.post(
                f"{self.base_url}/cmd",
                json=cmd.model_dump(),
                timeout=self.timeout_sec,
            )
            if resp.status_code == 200:
                self.is_connected = True
                self.last_contact_time = time.time()
                data = resp.json()
                return CommandAck(
                    ack_seq=data.get("ack_seq", cmd.seq),
                    status=data.get("status", "OK"),
                    message=data.get("message", ""),
                )
        except requests.RequestException as e:
            logger.debug(f"ESP32 command dispatch error: {e}")
            self.is_connected = False

        return None

    def fetch_telemetry(self) -> Optional[TelemetryPacket]:
        """Poll the latest sensor readings and status from ESP32."""
        try:
            resp = requests.get(f"{self.base_url}/telemetry", timeout=self.timeout_sec)
            if resp.status_code == 200:
                self.is_connected = True
                self.last_contact_time = time.time()
                data = resp.json()
                pkt = TelemetryPacket(**data)
                self.last_telemetry = pkt
                return pkt
        except requests.RequestException:
            self.is_connected = False
        return None

    def emergency_stop(self) -> bool:
        """Send instant high-priority emergency stop command."""
        ack = self.send_command(action="EMERGENCY_STOP", speed=0, duration_ms=0)
        return ack is not None and ack.status == "OK"

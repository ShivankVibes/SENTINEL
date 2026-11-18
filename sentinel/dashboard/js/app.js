/**
 * SENTINEL Mission Control Dashboard Client
 */

document.addEventListener('DOMContentLoaded', () => {
  // UI Elements
  const modeBadge = document.getElementById('mode-badge');
  const stateBadge = document.getElementById('state-badge');
  const commsBadge = document.getElementById('comms-badge');
  const btnModeToggle = document.getElementById('btn-mode-toggle');
  const btnEmergency = document.getElementById('btn-emergency');
  const camFps = document.getElementById('cam-fps');
  const camSource = document.getElementById('cam-source');

  const valX = document.getElementById('val-x');
  const valY = document.getElementById('val-y');
  const valHeading = document.getElementById('val-heading');
  const valUltrasonic = document.getElementById('val-ultrasonic');
  const ultrasonicGauge = document.getElementById('ultrasonic-gauge');
  const valAction = document.getElementById('val-action');
  const valReason = document.getElementById('val-reason');
  const valTargetName = document.getElementById('val-target-name');
  const valTargetCoords = document.getElementById('val-target-coords');
  const logConsole = document.getElementById('log-console');
  const btnClearLogs = document.getElementById('btn-clear-logs');

  // Initialize Tactical Map
  const arenaMap = new ArenaMapRenderer('arena-canvas');

  let currentMode = 'SIMULATION';
  let ws = null;
  let lastLogsCount = 0;

  // Setup Map Click Handler (Dispatches Target in Simulation Mode)
  arenaMap.onTargetSelected = (xCm, yCm) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'SET_TARGET',
        x_cm: xCm,
        y_cm: yCm
      }));
    }
  };

  // Connect WebSocket
  function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      commsBadge.textContent = 'COMMS: CONNECTED';
      commsBadge.className = 'badge badge-comms';
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        updateDashboard(data);
      } catch (err) {
        console.error('Telemetry parse error:', err);
      }
    };

    ws.onclose = () => {
      commsBadge.textContent = 'COMMS: DISCONNECTED';
      commsBadge.className = 'badge badge-stop';
      setTimeout(connectWebSocket, 2000);
    };

    ws.onerror = (err) => {
      console.error('WebSocket error:', err);
    };
  }

  function updateDashboard(data) {
    // 1. Mode Badge
    currentMode = data.mode;
    if (data.mode === 'SIMULATION') {
      modeBadge.textContent = 'MODE: SIMULATION';
      modeBadge.className = 'badge badge-sim';
      btnModeToggle.textContent = 'SWITCH TO REAL HARDWARE';
    } else {
      modeBadge.textContent = 'MODE: REAL HARDWARE';
      modeBadge.className = 'badge badge-real';
      btnModeToggle.textContent = 'SWITCH TO SIMULATION';
    }

    // 2. State Badge
    stateBadge.textContent = `STATE: ${data.system_state}`;
    if (data.system_state === 'EMERGENCY_STOP') {
      stateBadge.className = 'badge badge-stop';
      btnEmergency.textContent = 'CLEAR EMERGENCY';
      btnEmergency.classList.add('emergency-active');
    } else if (data.system_state === 'NAVIGATING') {
      stateBadge.className = 'badge badge-nav';
      btnEmergency.textContent = 'EMERGENCY STOP';
      btnEmergency.classList.remove('emergency-active');
    } else {
      stateBadge.className = 'badge badge-idle';
      btnEmergency.textContent = 'EMERGENCY STOP';
      btnEmergency.classList.remove('emergency-active');
    }

    // 3. Camera Status
    if (data.camera_status) {
      camFps.textContent = `FPS: ${data.camera_status.actual_fps}`;
      camSource.textContent = data.camera_status.is_synthetic
        ? 'Synthetic Arena Feed'
        : `Hardware Index ${data.camera_status.device_index}`;
    }

    // 4. Rover Coordinates & Heading
    if (data.rover_pose) {
      valX.textContent = data.rover_pose.x_cm.toFixed(1);
      valY.textContent = data.rover_pose.y_cm.toFixed(1);
      valHeading.textContent = data.rover_pose.heading_deg.toFixed(1);
    } else {
      valX.textContent = '--';
      valY.textContent = '--';
      valHeading.textContent = '--';
    }

    // 5. Ultrasonic Sensor
    if (data.ultrasonic_distance_cm !== undefined && data.ultrasonic_distance_cm >= 0) {
      valUltrasonic.textContent = data.ultrasonic_distance_cm.toFixed(1);
      const pct = Math.min(100, Math.max(0, (data.ultrasonic_distance_cm / 150.0) * 100));
      ultrasonicGauge.style.width = `${pct}%`;
      ultrasonicGauge.style.backgroundColor = pct < 20 ? '#ff1744' : (pct < 50 ? '#ffab00' : '#00e676');
    } else {
      valUltrasonic.textContent = '--';
      ultrasonicGauge.style.width = '0%';
    }

    // 6. Action / Instruction
    if (data.last_command) {
      valAction.textContent = data.last_command.action;
      valReason.textContent = data.last_command.reason || '';
    } else {
      valAction.textContent = 'IDLE';
      valReason.textContent = 'Awaiting instructions';
    }

    // 7. Hazard Information
    if (data.hazards && data.hazards.length > 0) {
      const h = data.hazards[0];
      valTargetName.textContent = `${h.label} (#${h.id})`;
      valTargetCoords.textContent = `X: ${h.x_cm.toFixed(1)} cm, Y: ${h.y_cm.toFixed(1)} cm [${h.detection_type}]`;
    } else {
      valTargetName.textContent = 'NONE DETECTED';
      valTargetCoords.textContent = 'X: -- cm, Y: -- cm';
    }

    // 8. Event Log Stream
    if (data.event_logs && data.event_logs.length !== lastLogsCount) {
      lastLogsCount = data.event_logs.length;
      logConsole.innerHTML = '';
      data.event_logs.forEach((item) => {
        const div = document.createElement('div');
        div.className = `log-entry log-${item.level.toLowerCase()}`;
        div.innerHTML = `<span class="log-time">[${item.timestamp}]</span> <span class="log-cat">[${item.category}]</span> <span class="log-msg">${item.message}</span>`;
        logConsole.appendChild(div);
      });
      logConsole.scrollTop = logConsole.scrollHeight;
    }

    // 9. Update Tactical Map
    arenaMap.render(data);
  }

  // Toggle Mode
  btnModeToggle.addEventListener('click', () => {
    const targetMode = currentMode === 'SIMULATION' ? 'REAL' : 'SIMULATION';
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'SET_MODE',
        mode: targetMode
      }));
    }
  });

  // Emergency Stop / Reset
  btnEmergency.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      if (btnEmergency.classList.contains('emergency-active')) {
        ws.send(JSON.stringify({ type: 'RESET_EMERGENCY' }));
      } else {
        ws.send(JSON.stringify({ type: 'EMERGENCY_STOP' }));
      }
    }
  });

  // Clear Logs
  btnClearLogs.addEventListener('click', () => {
    logConsole.innerHTML = '';
  });

  connectWebSocket();
});

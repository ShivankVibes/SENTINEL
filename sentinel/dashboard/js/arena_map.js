/**
 * Tactical Arena Map Renderer for SENTINEL Mission Control
 */

class ArenaMapRenderer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.arenaWidthCm = 200.0;
    this.arenaHeightCm = 150.0;
    this.pulseTick = 0;

    // Interaction callback
    this.onTargetSelected = null;

    this.initEvents();
  }

  initEvents() {
    this.canvas.addEventListener('click', (e) => {
      const rect = this.canvas.getBoundingClientRect();
      const scaleX = this.canvas.width / rect.width;
      const scaleY = this.canvas.height / rect.height;
      const clickPxX = (e.clientX - rect.left) * scaleX;
      const clickPxY = (e.clientY - rect.top) * scaleY;

      // Convert pixel to cm
      const padX = this.canvas.width * 0.08;
      const padY = this.canvas.height * 0.08;
      const activeW = this.canvas.width - 2 * padX;
      const activeH = this.canvas.height - 2 * padY;

      if (clickPxX >= padX && clickPxX <= padX + activeW &&
          clickPxY >= padY && clickPxY <= padY + activeH) {
        const xCm = ((clickPxX - padX) / activeW) * this.arenaWidthCm;
        const yCm = ((clickPxY - padY) / activeH) * this.arenaHeightCm;
        if (this.onTargetSelected) {
          this.onTargetSelected(Math.round(xCm * 10) / 10, Math.round(yCm * 10) / 10);
        }
      }
    });
  }

  cmToCanvas(xCm, yCm) {
    const padX = this.canvas.width * 0.08;
    const padY = this.canvas.height * 0.08;
    const activeW = this.canvas.width - 2 * padX;
    const activeH = this.canvas.height - 2 * padY;

    const px = padX + (xCm / this.arenaWidthCm) * activeW;
    const py = padY + (yCm / this.arenaHeightCm) * activeH;
    return [px, py];
  }

  render(telemetry) {
    this.pulseTick += 0.05;
    const ctx = this.ctx;
    const w = this.canvas.width;
    const h = this.canvas.height;

    // Clear
    ctx.fillStyle = '#0b0e14';
    ctx.fillRect(0, 0, w, h);

    const padX = w * 0.08;
    const padY = h * 0.08;
    const activeW = w - 2 * padX;
    const activeH = h - 2 * padY;

    // 1. Draw Arena Boundary Wall
    ctx.strokeStyle = '#26334d';
    ctx.lineWidth = 2;
    ctx.strokeRect(padX, padY, activeW, activeH);

    // 2. Draw Metric Grid (every 20cm)
    ctx.strokeStyle = '#182030';
    ctx.lineWidth = 1;
    for (let x = 0; x <= this.arenaWidthCm; x += 20) {
      const [gx, _] = this.cmToCanvas(x, 0);
      ctx.beginPath();
      ctx.moveTo(gx, padY);
      ctx.lineTo(gx, padY + activeH);
      ctx.stroke();
    }
    for (let y = 0; y <= this.arenaHeightCm; y += 20) {
      const [_, gy] = this.cmToCanvas(0, y);
      ctx.beginPath();
      ctx.moveTo(padX, gy);
      ctx.lineTo(padX + activeW, gy);
      ctx.stroke();
    }

    // 3. Draw Planned Path
    if (telemetry.active_path && telemetry.active_path.length > 1) {
      ctx.strokeStyle = '#00e676';
      ctx.lineWidth = 2.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      telemetry.active_path.forEach((pt, idx) => {
        const [px, py] = this.cmToCanvas(pt[0], pt[1]);
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.setLineDash([]);

      // Waypoint dots
      telemetry.active_path.forEach((pt) => {
        const [px, py] = this.cmToCanvas(pt[0], pt[1]);
        ctx.fillStyle = '#00e676';
        ctx.beginPath();
        ctx.arc(px, py, 4, 0, Math.PI * 2);
        ctx.fill();
      });
    }

    // 4. Draw Detected Hazards
    if (telemetry.hazards && telemetry.hazards.length > 0) {
      telemetry.hazards.forEach((h) => {
        const [hx, hy] = this.cmToCanvas(h.x_cm, h.y_cm);
        const radiusPx = (h.radius_cm / this.arenaWidthCm) * activeW;

        // Pulsing alert ring
        const pulse = 1 + 0.15 * Math.sin(this.pulseTick);
        ctx.strokeStyle = 'rgba(255, 23, 68, 0.4)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(hx, hy, radiusPx * pulse, 0, Math.PI * 2);
        ctx.stroke();

        // Hard circle
        ctx.fillStyle = 'rgba(255, 23, 68, 0.25)';
        ctx.beginPath();
        ctx.arc(hx, hy, radiusPx, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#ff1744';
        ctx.stroke();

        // Label
        ctx.fillStyle = '#ff1744';
        ctx.font = '10px monospace';
        ctx.fillText(`HAZARD #${h.id} (${h.x_cm}, ${h.y_cm})`, hx - 30, hy - radiusPx - 5);
      });
    }

    // 5. Draw Rover
    if (telemetry.rover_pose) {
      const pose = telemetry.rover_pose;
      const [rx, ry] = this.cmToCanvas(pose.x_cm, pose.y_cm);
      const roverRadPx = (9.0 / this.arenaWidthCm) * activeW; // 9cm radius

      // Rover chassis circle
      ctx.fillStyle = '#00d2ff';
      ctx.beginPath();
      ctx.arc(rx, ry, roverRadPx, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Heading arrow
      const rad = (pose.heading_deg * Math.PI) / 180.0;
      const arrowLen = roverRadPx * 1.8;
      const ax = rx + arrowLen * Math.cos(rad);
      const ay = ry + arrowLen * Math.sin(rad);

      ctx.strokeStyle = '#ffab00';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(ax, ay);
      ctx.stroke();

      // Label
      ctx.fillStyle = '#00d2ff';
      ctx.font = '10px monospace';
      ctx.fillText(`ROVER (${pose.x_cm}, ${pose.y_cm})`, rx - 25, ry + roverRadPx + 14);
    }
  }
}

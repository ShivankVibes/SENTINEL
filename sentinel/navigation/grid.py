"""Occupancy grid mapping and obstacle inflation module for SENTINEL."""

import math
from typing import Tuple, List, Optional
import numpy as np


class OccupancyGrid:
    """Discretized 2D grid representation of the demonstration floor."""

    def __init__(
        self,
        width_cm: float = 200.0,
        height_cm: float = 150.0,
        cell_size_cm: float = 5.0,
        safety_margin_cm: float = 15.0,
    ):
        self.width_cm = float(width_cm)
        self.height_cm = float(height_cm)
        self.cell_size_cm = float(cell_size_cm)
        self.safety_margin_cm = float(safety_margin_cm)

        self.cols = int(math.ceil(self.width_cm / self.cell_size_cm))
        self.rows = int(math.ceil(self.height_cm / self.cell_size_cm))

        # 0 = Free, 1 = Hard Obstacle, 2 = Inflated Safety Margin
        self.grid = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self.obstacles: List[Tuple[float, float, float]] = []  # (x_cm, y_cm, radius_cm)

    def clear(self):
        """Reset the occupancy grid to empty."""
        self.grid.fill(0)
        self.obstacles.clear()

    def metric_to_grid(self, x_cm: float, y_cm: float) -> Tuple[int, int]:
        """Convert floor metric coordinates (x, y) cm to grid indices (col, row)."""
        col = int(math.floor(x_cm / self.cell_size_cm))
        row = int(math.floor(y_cm / self.cell_size_cm))
        # Clamp within grid bounds
        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))
        return col, row

    def grid_to_metric(self, col: int, row: int) -> Tuple[float, float]:
        """Convert grid indices (col, row) to center metric coordinates (x, y) cm."""
        x_cm = (col + 0.5) * self.cell_size_cm
        y_cm = (row + 0.5) * self.cell_size_cm
        return round(x_cm, 1), round(y_cm, 1)

    def is_cell_walkable(self, col: int, row: int) -> bool:
        """Check if grid cell is inside bounds and free of obstacles/margins."""
        if col < 0 or col >= self.cols or row < 0 or row >= self.rows:
            return False
        return self.grid[row, col] == 0

    def add_circular_obstacle(self, x_cm: float, y_cm: float, radius_cm: float):
        """Add a circular obstacle and inflate safety margin around it."""
        self.obstacles.append((x_cm, y_cm, radius_cm))
        total_radius_cm = radius_cm + self.safety_margin_cm

        # Bounding box of the obstacle in grid space
        min_col, min_row = self.metric_to_grid(x_cm - total_radius_cm, y_cm - total_radius_cm)
        max_col, max_row = self.metric_to_grid(x_cm + total_radius_cm, y_cm + total_radius_cm)

        for r in range(min_row, max_row + 1):
            for c in range(min_col, max_col + 1):
                cx, cy = self.grid_to_metric(c, r)
                dist = math.hypot(cx - x_cm, cy - y_cm)
                if dist <= radius_cm:
                    self.grid[r, c] = 1  # Hard obstacle
                elif dist <= total_radius_cm:
                    if self.grid[r, c] == 0:
                        self.grid[r, c] = 2  # Safety margin

    def add_boundary_margins(self):
        """Inflate arena boundary walls by the safety margin."""
        margin_cells = int(math.ceil(self.safety_margin_cm / self.cell_size_cm))
        for r in range(self.rows):
            for c in range(self.cols):
                if (
                    r < margin_cells
                    or r >= self.rows - margin_cells
                    or c < margin_cells
                    or c >= self.cols - margin_cells
                ):
                    if self.grid[r, c] == 0:
                        self.grid[r, c] = 2

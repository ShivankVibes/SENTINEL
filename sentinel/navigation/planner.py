"""A* path planning algorithm with safety inflation and line-of-sight waypoint pruning."""

import heapq
import math
from typing import List, Tuple, Optional
from sentinel.navigation.grid import OccupancyGrid


class AStarPlanner:
    """Plans collision-free trajectories on the discretized arena occupancy grid."""

    def __init__(self, grid: OccupancyGrid):
        self.grid = grid

    def plan(
        self, start_cm: Tuple[float, float], goal_cm: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        """Compute an optimal smoothed trajectory from start_cm to goal_cm in floor coordinates."""
        start_col, start_row = self.grid.metric_to_grid(start_cm[0], start_cm[1])
        goal_col, goal_row = self.grid.metric_to_grid(goal_cm[0], goal_cm[1])

        # If goal is inside an obstacle (e.g. inspecting a hazard), find closest walkable boundary cell
        if not self.grid.is_cell_walkable(goal_col, goal_row):
            fallback_goal = self._find_nearest_walkable_cell(goal_col, goal_row)
            if fallback_goal is None:
                return None
            goal_col, goal_row = fallback_goal

        # If start is inside an obstacle margin, find nearest walkable cell
        if not self.grid.is_cell_walkable(start_col, start_row):
            fallback_start = self._find_nearest_walkable_cell(start_col, start_row)
            if fallback_start is None:
                return None
            start_col, start_row = fallback_start

        # A* Search
        # 8 directions: dx, dy, cost
        directions = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (-1, -1, math.sqrt(2)),
        ]

        def heuristic(c1, r1, c2, r2):
            return math.hypot(c1 - c2, r1 - r2)

        start_node = (start_col, start_row)
        goal_node = (goal_col, goal_row)

        open_set = []
        heapq.heappush(open_set, (0.0, start_node))

        came_from = {}
        g_score = {start_node: 0.0}
        f_score = {start_node: heuristic(start_col, start_row, goal_col, goal_row)}
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_node:
                # Reconstruct raw grid path
                raw_path = [current]
                while current in came_from:
                    current = came_from[current]
                    raw_path.append(current)
                raw_path.reverse()

                # Convert to metric coordinates and prune waypoints
                metric_path = [self.grid.grid_to_metric(c, r) for (c, r) in raw_path]
                # Prepend exact start and append exact goal
                metric_path[0] = (round(start_cm[0], 1), round(start_cm[1], 1))
                metric_path[-1] = (round(goal_cm[0], 1), round(goal_cm[1], 1))
                return self._prune_waypoints(metric_path)

            visited.add(current)
            curr_col, curr_row = current

            for dc, dr, step_cost in directions:
                neighbor = (curr_col + dc, curr_row + dr)
                ncol, nrow = neighbor

                if not self.grid.is_cell_walkable(ncol, nrow):
                    continue

                # Prevent diagonal corner-cutting through obstacle corners
                if dc != 0 and dr != 0:
                    if not self.grid.is_cell_walkable(curr_col + dc, curr_row) or \
                       not self.grid.is_cell_walkable(curr_col, curr_row + dr):
                        continue

                tentative_g = g_score[current] + step_cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(ncol, nrow, goal_col, goal_row)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))

        return None  # Path not reachable

    def _find_nearest_walkable_cell(self, col: int, row: int, max_search_radius: int = 15) -> Optional[Tuple[int, int]]:
        """Breadth-first search for the nearest walkable cell around an obstacle."""
        queue = [(col, row)]
        visited = {(col, row)}

        while queue:
            curr_c, curr_r = queue.pop(0)
            if self.grid.is_cell_walkable(curr_c, curr_r):
                return (curr_c, curr_r)

            dist = max(abs(curr_c - col), abs(curr_r - row))
            if dist >= max_search_radius:
                continue

            for dc, dr in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
                nbr = (curr_c + dc, curr_r + dr)
                if nbr not in visited:
                    visited.add(nbr)
                    if 0 <= nbr[0] < self.grid.cols and 0 <= nbr[1] < self.grid.rows:
                        queue.append(nbr)
        return None

    def _prune_waypoints(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Line-of-sight path smoothing: removes redundant collinear and unblocked waypoints."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            furthest_visible_idx = current_idx + 1
            for check_idx in range(len(path) - 1, current_idx, -1):
                if self._has_line_of_sight(path[current_idx], path[check_idx]):
                    furthest_visible_idx = check_idx
                    break
            smoothed.append(path[furthest_visible_idx])
            current_idx = furthest_visible_idx

        return smoothed

    def _has_line_of_sight(self, pt1: Tuple[float, float], pt2: Tuple[float, float]) -> bool:
        """Raycast between two metric points to verify that no obstacle margins intersect."""
        dist = math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])
        steps = max(2, int(math.ceil(dist / (self.grid.cell_size_cm / 2.0))))
        for s in range(steps + 1):
            t = s / float(steps)
            x = pt1[0] + t * (pt2[0] - pt1[0])
            y = pt1[1] + t * (pt2[1] - pt1[1])
            c, r = self.grid.metric_to_grid(x, y)
            if not self.grid.is_cell_walkable(c, r):
                return False
        return True

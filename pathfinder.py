"""
pathfinder.py
==========================================================
A* Pathfinding Algorithm
----------------------------------------------------------
PARALLEL COMPUTING CONCEPT:
  This function runs INSIDE each drone's own process.
  Because every drone is a separate OS process, all
  drones compute their paths at the EXACT SAME TIME
  on different CPU cores - true parallel path planning.
==========================================================
"""

import heapq
from config import GRID_SIZE


def astar(start: tuple, goal: tuple, obstacles: set) -> list:
    """
    A* shortest path from start to goal, avoiding obstacles.

    Args:
        start     : (col, row) tuple - starting grid cell
        goal      : (col, row) tuple - target grid cell
        obstacles : set of (col, row) blocked cells

    Returns:
        List of (x, y) waypoints from start to goal.
        Falls back to [start, goal] if no path found.
    """
    if start == goal:
        return [start]

    def heuristic(a, b):
        # Diagonal (Chebyshev) distance - works well with 8-directional movement
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

    # 8-directional movement: cardinal + diagonal
    MOVES = [
        (0, 1), (0, -1), (1, 0), (-1, 0),
        (1, 1), (1, -1), (-1, 1), (-1, -1),
    ]

    open_heap = []
    heapq.heappush(open_heap, (0 + heuristic(start, goal), 0, start))

    came_from = {}
    g_cost    = {start: 0}

    while open_heap:
        _, cost, current = heapq.heappop(open_heap)

        if current == goal:
            # Reconstruct path
            path = []
            node = current
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in MOVES:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)

            # Bounds check
            if not (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE):
                continue
            # Obstacle check
            if neighbor in obstacles:
                continue

            # Diagonal moves cost sqrt2 ~ 1.414; cardinal cost 1.0
            step = 1.414 if (dx != 0 and dy != 0) else 1.0
            new_cost = g_cost[current] + step

            if new_cost < g_cost.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_cost[neighbor]    = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, new_cost, neighbor))

    # No path found - return straight line as fallback
    return [start, goal]

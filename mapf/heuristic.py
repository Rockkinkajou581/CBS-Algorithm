"""Backward BFS heuristic for space-time A*.

For a given goal cell, a single BFS run over the grid (ignoring time and
ignoring other agents, respecting only static obstacles) gives the exact
shortest unobstructed distance from every cell to that goal. This is
admissible and consistent for space-time A*, and much tighter than
Manhattan distance whenever the grid has obstacles that force detours.

One BFS per distinct goal cell, cached and reused across every CT node's
re-plan of that agent in high_level.py (the heuristic doesn't change even
though constraints do).
"""

from collections import deque

from mapf.grid import Grid



def backward_bfs(grid: Grid, goal: tuple[int, int]) -> dict:
    """Return {cell: exact shortest-path distance to goal}.

    Unreachable cells should be absent (or mapped to float('inf')) so
    low_level.py can detect an infeasible instance early.
    """
    visited = {goal: 0}
    queue = deque([goal])

    while queue:
        cell = queue.popleft()
        dist = visited[cell]
        for neighbor in grid.neighbors(cell):
            if neighbor not in visited:
                visited[neighbor] = dist + 1
                queue.append(neighbor)

    return visited


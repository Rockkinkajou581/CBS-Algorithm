"""
BFS hueristic for A star
"""

from collections import deque

from mapf.grid import Grid



def backward_bfs(grid: Grid, goal: tuple[int, int]) -> dict:
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


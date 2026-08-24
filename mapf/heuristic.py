"""
BFS hueristic for A star
"""

from collections import deque

from mapf.grid import Grid



def backward_bfs(grid: Grid, goal: tuple[int, int]) -> dict:
    visited = {goal: 0}
    queue = deque([goal])
    #Standard BFS, pop off queue
    while queue:
        cell = queue.popleft()
        dist = visited[cell]
        #search all unvisited neighbors
        for neighbor in grid.neighbors(cell):
            if neighbor not in visited:
                #store distance in the visited dictionary as discovered
                visited[neighbor] = dist + 1
                queue.append(neighbor)

    return visited


"""Single-agent search in space-time. Implementation of A*
"""

from mapf.constraints import EdgeConstraint, VertexConstraint
from mapf.grid import Grid
import heapq
from mapf.heuristic import backward_bfs

def space_time_astar(
    grid: Grid,
    start: tuple[int, int],
    goal: tuple[int, int],
    agent: int,
    vertex_constraints: set[VertexConstraint],
    edge_constraints: set[EdgeConstraint],
    max_constraint_time: int
) -> list[tuple[int, int]] | None:

    h = backward_bfs(grid, goal)
    start_state = (start, 0)
    if start not in h:
      return None

    g = {}
    g[start_state] = 0
    heap = []
    came_from = {}
    closed = set()

    heapq.heappush(heap, (h[start], start_state))
    while True:
      priority, (cell, t) = heapq.heappop(heap) #take apart the popped state
      pop = (cell, t)
      if pop in closed:
        continue
      closed.add(pop)
      if cell == goal and t >= max_constraint_time:
        return unravel(came_from, (goal, t))
      for neighbor_cell in grid.neighbors(cell) + [cell]:
        state = (neighbor_cell, t+1)
        if VertexConstraint(agent, neighbor_cell, t + 1) in vertex_constraints:
          continue
        if EdgeConstraint(agent, cell, neighbor_cell, t) in edge_constraints:
          continue
        canidate = g[pop] + 1
        if state not in g or canidate < g[state]:
          g[state] = canidate
          heapq.heappush(heap, (h[neighbor_cell] + canidate, state))
          came_from[state] = pop


def unravel(came_from: dict, goal):
  path = [goal[0]]
  curr = goal
  while (True):
    curr = came_from.get(curr)
    if(curr is None):
      break
    path.append(curr[0])
  path.reverse()

  return path

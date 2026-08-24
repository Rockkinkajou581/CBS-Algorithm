"""
CBS high-level search, see README
"""

import heapq
import itertools
from mapf.constraints import EdgeConstraint, VertexConstraint
from mapf.grid import Grid
from mapf.low_level import space_time_astar

"""One node of the constraint tree"""
class CTNode:
    def __init__(self, constraints, solution, cost):
      self.constraints = constraints
      self.solution = solution
      self.cost = cost

def get_loc(path, t):
  if t < len(path):
    return path[t]
  else:
    return path[-1]

def get_edge(path, t):
  if(t < len(path) - 1):
    return (path[t], path[t + 1])
  else:
    return (path[-1], path[-1])


"""
Finds conflicts. Can update to add generalized conflict. Conflict_sets is a set containing {conVV, conE, conVE}, the extra conflicts
"""
def find_conflict(solution: dict):
    #after reached path conflicts 
    max_t = max([len(x) for x in solution.values()])
    for t in range(max_t):
      seen = {} #vertexs
      edges = {}
      for agent, path in solution.items():
        
        #vertex conflict check
        loc = get_loc(path, t)
        if seen.get(loc) is None:
          seen[loc] = agent 
        else:
          return (seen[loc], agent, loc, t)

        #edge conflict check
        (a, b) = get_edge(path, t)

        if(a == b):
          continue

        if edges.get((b, a)) is None:
          edges[(a,b)] = agent
        else:
          return (edges[(b,a)], agent, (a, b), t)
    return None
"""Get cost of a solution by summing all lenghts"""   
def cost(solution: dict):
  sum = 0 
  for agent, path in solution.items():
    sum += (len(path) - 1)
  return sum 

"""Helper function to seperate out edge and vertex constraints into different sets"""
def separate_Vertex_and_Edge(constraint: set) -> tuple:
  v_constraints = set()
  e_constraints = set()
  for item in constraint:
    if isinstance(item, VertexConstraint):
      v_constraints.add(item)
    elif isinstance(item, EdgeConstraint):
      e_constraints.add(item)
  return (v_constraints, e_constraints)

"""Main function to create a new CT node"""
def generate_one_node(grid: Grid, agents: dict, conflict: tuple, agenta: bool, new_node: CTNode, current_node: CTNode):
  agent_a, agent_b, loc_or_edge, t = conflict
  if isinstance(loc_or_edge[0], tuple):
    (loc_from, loc_to) = loc_or_edge
    added_constraint = EdgeConstraint(agent_a, loc_to, loc_from, t) if agenta else EdgeConstraint(agent_b, loc_from, loc_to, t)
  else:
    added_constraint = VertexConstraint(agent_a, loc_or_edge, t) if agenta else VertexConstraint(agent_b, loc_or_edge, t)

  #add constraint from found conflict
  new_node.constraints = current_node.constraints | {added_constraint}

  (v, e) = separate_Vertex_and_Edge(new_node.constraints)
  new_node.solution = current_node.solution.copy()

  #then solve with the new constraints
  if agenta:
    start, goal = agents[agent_a]
    max_time_constraint = max([vertex.time for vertex in v if vertex.agent == agent_a], default=0)
    new_node.solution[agent_a] = space_time_astar(grid, start, goal, agent_a, v, e, max_time_constraint)
  else:
    start, goal = agents[agent_b]
    max_time_constraint = max([vertex.time for vertex in v if vertex.agent == agent_b], default=0)
    new_node.solution[agent_b] = space_time_astar(grid, start, goal, agent_b, v, e, max_time_constraint)
  new_node.cost = cost(new_node.solution)

"""Main CBS function. Returns dict[agents] -> list[nodes] (path per agent)"""
def conflict_based_search(grid: Grid, agents: dict) -> dict | None:

    counter = itertools.count()
    first_solution = dict()
    #start solution
    for agent, (start, goal) in agents.items():
      first_solution[agent] = space_time_astar(grid, start, goal, agent, set(), set(), 0)
      if first_solution[agent] is None:
        return None
    root_node = CTNode(set(), first_solution, cost(first_solution))
    open = []
    heapq.heappush(open, (root_node.cost, next(counter), root_node))
    while True:
      priority, count, node = heapq.heappop(open)
      #get conflict
      conflict = find_conflict(node.solution)

      if conflict is None:
        return node.solution
      #make both children
      N1 = CTNode(set(), dict(), 0)
      N2 = CTNode(set(), dict(), 0)
      generate_one_node(grid, agents, conflict, True, N1, node)
      generate_one_node(grid, agents, conflict, False, N2, node)

      heapq.heappush(open, (N1.cost, next(counter), N1))
      heapq.heappush(open, (N2.cost, next(counter), N2))


    
# CBS: Conflict-Based Search for Multi-Agent Pathfinding

A Python implementation of **Conflict-Based Search (CBS)**, an optimal algorithm for the Multi-Agent Path Finding (MAPF) problem: given a grid, a set of agents, and a start/goal cell for each, find a set of paths per agent that are shortest and free of collisions.

Implementation of: [Sharon, Stern, Felner & Sturtevant, *"Conflict-Based Search For Optimal Multi-Agent Pathfinding"*](https://www.sciencedirect.com/science/article/pii/S0004370214001386).

## How it works

CBS is a two-level algorithm:

- **High level** ([`high_level.py`](mapf/high_level.py)) searches a *constraint tree* (CT). Each node holds one path per agent and the set of constraints that produced them. Starting from an unconstrained root, it repeatedly pops the lowest-cost node, checks the joint solution for the first conflict between any two agents, and branches into two children, each forbidding one of the two agents from the conflicting move. The search ends when a node's joint solution is completely conflict-free; because nodes are expanded in cost order, that solution must be optimal.

- **Low level** ([`low_level.py`](mapf/low_level.py)) plans a single agent's path with `space_time_astar`, an A* search over *(cell, timestep)* states.

Two conflict types are detected and resolved:
- **Vertex conflicts** — two agents occupy the same cell at the same time
- **Edge conflicts** — two agents swap cells across the same timestep

## Project structure

| File | Responsibility |
|---|---|
| [`mapf/grid.py`](mapf/grid.py) | Static grid consisting of bounds, obstacles, 4-connected neighbors |
| [`mapf/heuristic.py`](mapf/heuristic.py) | Backward BFS from each goal, used as heuristic for A* |
| [`mapf/constraints.py`](mapf/constraints.py) | `VertexConstraint` / `EdgeConstraint` — what the high level hands to the low level |
| [`mapf/low_level.py`](mapf/low_level.py) | `space_time_astar` — A* for finding shortest path of a single agent|
| [`mapf/high_level.py`](mapf/high_level.py) | `conflict_based_search` — the constraint-tree search over joint solutions |

## Usage

```python
from mapf.grid import Grid
from mapf.high_level import conflict_based_search

grid = Grid(width=5, height=5, obstacles=frozenset())

agents = {
    0: ((0, 0), (4, 4)),  # agent 0: start -> goal
    1: ((4, 0), (0, 4)),  # agent 1: start -> goal
}

solution = conflict_based_search(grid, agents)
# solution: dict[agent_id] -> list of (x, y) cells
```
## Issues:
**No solution** - code breaks if, say, you put two robots in a 1 wide room, going head on. The original paper uses a check for this. 

**Unoptimal Hueristic calling** - BFS is rerun everytime in low_level when it doesn't need to and instead can be precomputed and stored for each goal. 


## AI Use:
Function outlines and signatures, some classes, and readme were outlined with Claude Code and test files and were genearted with AI. All core algorithms like high_level.py, low_level.py and heuristic.py are my own. 
"""
Unit tests for A*. Tests were generated with Claude Code
"""

import unittest

from mapf.constraints import EdgeConstraint, VertexConstraint
from mapf.grid import Grid
from mapf.low_level import space_time_astar


class TestSpaceTimeAStar(unittest.TestCase):
    def test_straight_line(self):
        # Empty 5x1 grid, no obstacles, no constraints. Shortest path is
        # a straight walk, no waits: length == Manhattan distance == 4.
        grid = Grid(width=5, height=1, obstacles=frozenset())
        path = space_time_astar(
            grid, (0, 0), (4, 0), agent=0,
            vertex_constraints=set(), edge_constraints=set(),
            max_constraint_time=0,
        )
        self.assertEqual(path, [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)])

    def test_detour_around_wall(self):
        # 3x3 grid. Wall across y=1 except a gap at x=2:
        #   y=2: . . .
        #   y=1: # # .   <- obstacles at (0,1) and (1,1)
        #   y=0: . . .
        # Only route from (0,0) to (0,2) is through the gap at (2,1),
        # a hand-countable 6-step detour.
        grid = Grid(width=3, height=3, obstacles=frozenset({(0, 1), (1, 1)}))
        path = space_time_astar(
            grid, (0, 0), (0, 2), agent=0,
            vertex_constraints=set(), edge_constraints=set(),
            max_constraint_time=0,
        )
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (0, 2))
        self.assertEqual(len(path) - 1, 6)

    def test_vertex_constraint_forces_wait(self):
        # Same 5x1 corridor as test_straight_line, but the agent is
        # forbidden from being at (1,0) at t=1 -- the second cell on its
        # natural path, at the moment it would naturally arrive there.
        # With no alternative cell to detour through (1D corridor), the
        # only way around is one extra wait at the start before moving.
        grid = Grid(width=5, height=1, obstacles=frozenset())
        vc = {VertexConstraint(agent=0, loc=(1, 0), time=1)}
        path = space_time_astar(
            grid, (0, 0), (4, 0), agent=0,
            vertex_constraints=vc, edge_constraints=set(),
            max_constraint_time=0,
        )
        self.assertEqual(
            path, [(0, 0), (0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        )

    def test_edge_constraint_blocks_swap(self):
        # Same corridor, but the specific transition (0,0)->(1,0) at
        # t=0 is forbidden (not the vertex, the edge itself). Same forced
        # detour as the vertex case: one wait, then proceed -- the
        # transition at t=1 onward is untouched by this constraint.
        grid = Grid(width=5, height=1, obstacles=frozenset())
        ec = {EdgeConstraint(agent=0, loc_from=(0, 0), loc_to=(1, 0), time=0)}
        path = space_time_astar(
            grid, (0, 0), (4, 0), agent=0,
            vertex_constraints=set(), edge_constraints=ec,
            max_constraint_time=0,
        )
        self.assertEqual(
            path, [(0, 0), (0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        )

    def test_goal_hold_constraint(self):
        # 2x1 grid: start (0,0), goal (1,0). Natural arrival is t=1, but
        # the agent is forbidden from being at the goal exactly at t=1.
        # There's no other cell to detour through, so the only legal
        # route is: wait once at start, then move -- arriving at t=2,
        # which clears the constraint (max_constraint_time=1 means the
        # agent must hold the goal from t=1 onward, but t=1 itself is
        # blocked, so t=2 is the earliest legal arrival).
        grid = Grid(width=2, height=1, obstacles=frozenset())
        vc = {VertexConstraint(agent=0, loc=(1, 0), time=1)}
        path = space_time_astar(
            grid, (0, 0), (1, 0), agent=0,
            vertex_constraints=vc, edge_constraints=set(),
            max_constraint_time=1,
        )
        self.assertEqual(path, [(0, 0), (0, 0), (1, 0)])

    def test_infeasible_returns_none(self):
        # Goal cell (1,2) is fully walled in on a 3x3 grid -- no path
        # exists at all, ignoring time entirely. Must return None, not
        # raise or hang.
        grid = Grid(
            width=3, height=3,
            obstacles=frozenset({(1, 0), (0, 1), (1, 1), (2, 1)}),
        )
        path = space_time_astar(
            grid, (0, 0), (1, 2), agent=0,
            vertex_constraints=set(), edge_constraints=set(),
            max_constraint_time=0,
        )
        self.assertIsNone(path)


if __name__ == "__main__":
    unittest.main()

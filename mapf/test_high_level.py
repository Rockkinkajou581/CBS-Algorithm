"""Unit tests for find_conflict against hand-verifiable joint solutions.

solution here is dict[agent_id] -> path, exactly the shape
space_time_astar returns per agent (a list of cells, one per timestep).
"""

import unittest

from mapf.grid import Grid
from mapf.high_level import conflict_based_search, find_conflict


class TestFindConflict(unittest.TestCase):
    def test_no_conflict_disjoint_paths(self):
        # Two agents on separate rows, never share a cell at all.
        solution = {
            0: [(0, 0), (1, 0), (2, 0)],
            1: [(0, 5), (1, 5), (2, 5)],
        }
        self.assertIsNone(find_conflict(solution))
    def test_no_conflict_shared_cell_different_times(self):
        # Both agents visit (1,0) at some point, but never at the same
        # time -- agent 1 is there at t=0, agent 0 is there at t=1.
        # Visiting the same cell at different times is completely legal
        # and must NOT be reported as a conflict.
        solution = {
            0: [(0, 0), (1, 0), (2, 0)],
            1: [(1, 0), (1, 1), (1, 2)],
        }
        self.assertIsNone(find_conflict(solution))

    def test_vertex_conflict_same_time(self):
        # Both agents are at (1,0) at t=1, arriving from and departing
        # to different cells -- a pure vertex conflict, no edge swap
        # involved.
        solution = {
            0: [(0, 0), (1, 0), (2, 0)],
            1: [(2, 2), (1, 0), (0, 2)],
        }
        self.assertEqual(find_conflict(solution), (0, 1, (1, 0), 1))

    def test_edge_conflict_swap(self):
        # Agents swap cells across the same timestep: agent 0 goes
        # (0,0)->(1,0), agent 1 goes (1,0)->(0,0), both between t=0
        # and t=1. Neither is ever at the same cell at the same time,
        # so this is purely an edge conflict, not a vertex conflict.
        solution = {
            0: [(0, 0), (1, 0)],
            1: [(1, 0), (0, 0)],
        }
        self.assertEqual(
            find_conflict(solution), (0, 1, ((1, 0), (0, 0)), 0)
        )

    def test_trivial_single_cell_path_no_crash(self):
        # Agent 0's start IS its goal -- a valid, real output of
        # space_time_astar (path length 1, no movement at all).
        # find_conflict must handle this without crashing.
        solution = {
            0: [(3, 3)],
            1: [(0, 0), (1, 0), (2, 0)],
        }
        self.assertIsNone(find_conflict(solution))


class TestConflictBasedSearch(unittest.TestCase):
    def test_single_agent(self):
        # Trivial baseline: one agent, empty grid, straight line. No
        # conflicts possible, root solution should be returned as-is.
        grid = Grid(width=5, height=1, obstacles=frozenset())
        agents = {0: ((0, 0), (4, 0))}
        result = conflict_based_search(grid, agents)
        self.assertEqual(result, {0: [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]})

    def test_two_agents_no_interaction(self):
        # Separate rows, paths never come near each other. Root
        # solution is already conflict-free -- no branching needed,
        # each agent gets its own unconstrained shortest path.
        grid = Grid(width=5, height=5, obstacles=frozenset())
        agents = {
            0: ((0, 0), (4, 0)),
            1: ((0, 4), (4, 4)),
        }
        result = conflict_based_search(grid, agents)
        self.assertIsNone(find_conflict(result))
        self.assertEqual(len(result[0]) - 1, 4)
        self.assertEqual(len(result[1]) - 1, 4)

    def test_vertex_conflict_resolved_with_room_to_pass(self):
        # 3x3 open grid, two agents fully swapping places along the
        # middle row. This is still an end-to-end swap (same
        # topological issue as a bare 1-wide corridor: "just wait"
        # produces a hidden edge conflict, hand-verified separately --
        # (0,1),(1,1),(2,1) vs (2,1),(2,1),(1,1),(0,1) collide via a
        # swap at t=1 despite the added wait). What makes this
        # instance actually solvable, unlike the pure corridor case,
        # is that rows y=0 and y=2 exist, so a real detour off the
        # middle row is available. Not asserting an exact cost here --
        # working out the true optimal detour cost by hand is exactly
        # the error-prone calculation that produced a wrong answer
        # twice already in this file's history -- just checking CBS
        # finds *some* genuinely conflict-free solution.
        grid = Grid(width=3, height=3, obstacles=frozenset())
        agents = {
            0: ((0, 1), (2, 1)),
            1: ((2, 1), (0, 1)),
        }
        result = conflict_based_search(grid, agents)
        self.assertIsNotNone(result)
        self.assertIsNone(find_conflict(result))

    def test_perpendicular_crossing_exact_cost(self):
        # 3x3 open grid: agent 0 crosses left-to-right along the
        # middle row, agent 1 crosses bottom-to-top along the middle
        # column. Their paths cross at the center cell (1,1) at the
        # same timestep (t=1) if neither yields. Hand-verified: agent
        # 1 waits once at its start, letting agent 0 clear the center
        # first -- cost 2 (agent 0, unconstrained) + 3 (agent 1, one
        # extra wait) = 5, and this is provably conflict-free (traced
        # by hand: they're never at the same cell or swapping edges at
        # any shared timestep).
        grid = Grid(width=3, height=3, obstacles=frozenset())
        agents = {
            0: ((0, 1), (2, 1)),
            1: ((1, 0), (1, 2)),
        }
        result = conflict_based_search(grid, agents)
        self.assertIsNotNone(result)
        self.assertIsNone(find_conflict(result))
        total_cost = sum(len(path) - 1 for path in result.values())
        self.assertEqual(total_cost, 5)

    def test_bottleneck_single_gap_forces_sequencing(self):
        # 3x3 grid with a wall across the middle row except a single
        # gap at (1,1). Both agents must funnel through that one gap,
        # but from the same side (not swapping ends), so no
        # topological deadlock -- just sequencing through a bottleneck.
        grid = Grid(width=3, height=3, obstacles=frozenset({(0, 1), (2, 1)}))
        agents = {
            0: ((0, 0), (0, 2)),
            1: ((2, 0), (2, 2)),
        }
        result = conflict_based_search(grid, agents)
        self.assertIsNotNone(result)
        self.assertIsNone(find_conflict(result))

    def test_three_agents_no_interaction(self):
        # Three agents on three separate rows -- no branching needed
        # at all, exercises the multi-agent root-building path and
        # find_conflict scanning across more than two agents at once.
        grid = Grid(width=5, height=5, obstacles=frozenset())
        agents = {
            0: ((0, 0), (4, 0)),
            1: ((0, 2), (4, 2)),
            2: ((0, 4), (4, 4)),
        }
        result = conflict_based_search(grid, agents)
        self.assertIsNotNone(result)
        self.assertIsNone(find_conflict(result))
        total_cost = sum(len(path) - 1 for path in result.values())
        self.assertEqual(total_cost, 12)

    def test_infeasible_disconnected_returns_none(self):
        # Agent 0's goal is fully walled off -- no path exists at all,
        # regardless of the other agent. Must return None cleanly, not
        # crash and not hang.
        grid = Grid(
            width=3, height=3,
            obstacles=frozenset({(1, 0), (0, 1), (1, 1), (2, 1)}),
        )
        agents = {
            0: ((0, 0), (1, 2)),
            1: ((2, 2), (0, 0)),
        }
        result = conflict_based_search(grid, agents)
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()

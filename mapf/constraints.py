"""Constraint types used to prune space-time A* in low_level.py.

These are what high_level.py attaches to a CT node's constraint set when
it branches on a conflict: given two agents colliding at a cell/edge and
time, one child node gets a constraint against agent A, the other against
agent B.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class VertexConstraint:
    """Agent may not occupy `loc` at time `time`."""
    agent: int
    loc: tuple[int, int]
    time: int


@dataclass(frozen=True)
class EdgeConstraint:
    """Agent may not move loc_from -> loc_to between time and time+1.

    Forbids the specific directed traversal, so it also blocks the swap
    conflict where two agents cross the same edge in opposite directions.
    """
    agent: int
    loc_from: tuple[int, int]
    loc_to: tuple[int, int]
    time: int

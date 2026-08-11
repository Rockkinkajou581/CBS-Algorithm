
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
    """
    agent: int
    loc_from: tuple[int, int]
    loc_to: tuple[int, int]
    time: int

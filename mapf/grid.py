"""
Grid holding (x,y) locations within width and height. Returns the neighbors.
"""

from dataclasses import dataclass

# 4-connected moves, in (dx, dy). No diagonals.
_MOVES = ((1, 0), (-1, 0), (0, 1), (0, -1))


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    obstacles: frozenset  # frozenset[tuple[int, int]] of blocked cells

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: tuple[int, int]) -> bool:
        """in_bounds and not an obstacle."""
        return self.in_bounds(cell) and cell not in self.obstacles

    def neighbors(self, cell: tuple[int, int]) -> list[tuple[int, int]]:
        """4-connected neighbors that are in_bounds and free.

        Does NOT include the wait-in-place action — low_level.py adds
        that itself, since whether waiting is legal depends on
        constraints, not just the grid.
        """
        x, y = cell
        return [
            (x + dx, y + dy)
            for dx, dy in _MOVES
            if self.is_free((x + dx, y + dy))
        ]

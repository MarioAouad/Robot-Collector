from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Tuple, Set

Pos = Tuple[int, int]

@dataclass
class Grid:
    width: int
    height: int
    walls: Set[Pos] = field(default_factory=set)

    def in_bounds(self, p: Pos) -> bool:
        x, y = p
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, p: Pos) -> bool:
        return p not in self.walls

    def neighbors(self, p: Pos) -> List[Pos]:
        x, y = p
        cand = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        return [q for q in cand if self.in_bounds(q) and self.passable(q)]

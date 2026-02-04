from __future__ import annotations
from typing import Tuple
from grid import Pos

def h_manhattan(a: Pos, b: Pos) -> int:
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def h_euclidean(a: Pos, b: Pos) -> float:
    dx = a[0]-b[0]; dy = a[1]-b[1]
    return (dx*dx + dy*dy) ** 0.5

def h_chebyshev(a: Pos, b: Pos) -> int:
    return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

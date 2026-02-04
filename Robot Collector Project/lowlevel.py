from __future__ import annotations
from typing import Dict, Tuple, List, Callable, Generator
import heapq, math
from grid import Grid, Pos
from heuristics import h_manhattan

SearchEvent = Tuple[str, Dict]

def reconstruct(came_from: Dict[Pos, Pos], start: Pos, goal: Pos) -> List[Pos]:
    if goal not in came_from and goal != start:
        return []
    path = [goal]
    cur = goal
    while cur != start:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path

def ucs_path(grid: Grid, start: Pos, goal: Pos) -> List[Pos]:
    gen = ucs_search(grid, start, goal)
    try:
        while True:
            next(gen)
    except StopIteration as e:
        return e.value or []

def ucs_search(grid: Grid, start: Pos, goal: Pos):
    frontier: List[Tuple[int, Pos]] = []
    heapq.heappush(frontier, (0, start))
    g = {start: 0}
    came_from: Dict[Pos, Pos] = {}
    explored = set()

    while frontier:
        cost, cur = heapq.heappop(frontier)
        if cost != g.get(cur, math.inf):
            continue
        explored.add(cur)

        yield ('step', {
            'current': cur,
            'frontier': [pos for _, pos in frontier],
            'explored': list(explored),
        })

        if cur == goal:
            return reconstruct(came_from, start, goal)

        for nxt in grid.neighbors(cur):
            ng = g[cur] + 1
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                came_from[nxt] = cur
                heapq.heappush(frontier, (ng, nxt))
    return []

def astar_path(grid: Grid, start: Pos, goal: Pos, h: Callable[[Pos,Pos], float] = h_manhattan) -> List[Pos]:
    gen = astar_search(grid, start, goal, h)
    try:
        while True:
            next(gen)
    except StopIteration as e:
        return e.value or []

def astar_search(grid: Grid, start: Pos, goal: Pos, h: Callable[[Pos,Pos], float] = h_manhattan):
    frontier: List[Tuple[float, int, Pos]] = []
    g = {start: 0.0}
    tie = 0
    heapq.heappush(frontier, (h(start, goal), tie, start))
    came_from: Dict[Pos, Pos] = {}
    explored = set()

    while frontier:
        f, _, cur = heapq.heappop(frontier)
        if f != g.get(cur, math.inf) + h(cur, goal):
            continue
        explored.add(cur)

        yield ('step', {
            'current': cur,
            'frontier': [pos for _,__,pos in frontier],
            'explored': list(explored),
        })

        if cur == goal:
            return reconstruct(came_from, start, goal)

        for nxt in grid.neighbors(cur):
            ng = g[cur] + 1
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                came_from[nxt] = cur
                tie += 1
                heapq.heappush(frontier, (ng + h(nxt, goal), tie, nxt))
    return []
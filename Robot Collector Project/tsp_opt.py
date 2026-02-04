from __future__ import annotations
from typing import List, Callable
import math
from grid import Grid, Pos
from lowlevel import ucs_path, astar_path
from heuristics import h_manhattan
def pairwise_distances(grid: Grid, points: List[Pos], algo: str = "astar", h: Callable[[Pos,Pos], float] = h_manhattan):
    n=len(points); D=[[math.inf]*n for _ in range(n)]; P=[[[] for _ in range(n)] for _ in range(n)]
    for i in range(n): D[i][i]=0; P[i][i]=[points[i]]
    for i in range(n):
        for j in range(i+1,n):
            s,g=points[i], points[j]
            path = ucs_path(grid,s,g) if algo=="ucs" else astar_path(grid,s,g,h=h)
            dist = math.inf if not path else len(path)-1
            D[i][j]=D[j][i]=dist; P[i][j]=path; P[j][i]=list(reversed(path)) if path else []
    if any(math.isinf(D[i][j]) for i in range(n) for j in range(n) if i!=j): return None, None
    return D,P
def optimal_order(points: List[Pos], D, home_idx: int=0, current_idx: int|None=None)->List[int]:
    n=len(points)
    if n==0: return []
    if current_idx is None: current_idx=home_idx
    order=[current_idx]; to_visit={i for i in range(n) if i!=current_idx and i!=home_idx}; cur=current_idx
    while to_visit:
        best=None; best_d=math.inf
        for j in to_visit:
            d=D[cur][j]; 
            if d<best_d: best_d=d; best=j
        if best is None or math.isinf(best_d): return []
        order.append(best); to_visit.remove(best); cur=best
    if cur!=home_idx:
        if math.isinf(D[cur][home_idx]): return []
        order.append(home_idx)
    return order

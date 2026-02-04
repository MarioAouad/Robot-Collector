from __future__ import annotations
import random, time
from typing import List, Set, Tuple, Dict, Callable
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception:
    tk = None

from grid import Grid, Pos
from heuristics import h_manhattan, h_euclidean, h_chebyshev
from lowlevel import ucs_search, astar_search
from tsp_opt import pairwise_distances, optimal_order

CELL = 22
PAD  = 10

HEURISTICS: Dict[str, Callable] = {
    "Manhattan": h_manhattan,
    "Euclidean": h_euclidean,
    "Chebyshev": h_chebyshev,
}

def fmt_time(seconds: float) -> str:
    if seconds < 0: seconds = 0
    m = int(seconds // 60)
    s = int(seconds % 60)
    ms = int((seconds - int(seconds)) * 1000)
    return f"{m:02d}:{s:02d}.{ms:03d}"

class App:
    def __init__(self):
        if tk is None:
            raise RuntimeError("Tkinter not available.")
        self.root = tk.Tk()
        self.root.title("Collector – v4 (no walls, scroll, straight path, robust restart, timer, trail, fix layers)")

        # Controls
        top = tk.Frame(self.root); top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=8)
        tk.Label(top, text="Grid").pack(side=tk.LEFT)
        self.grid_size = tk.StringVar(value="20")
        ttk.Combobox(top, textvariable=self.grid_size, values=["10","20","50"], state="readonly", width=5).pack(side=tk.LEFT, padx=6)

        tk.Label(top, text="Low-level Algo").pack(side=tk.LEFT, padx=(12,2))
        self.algo = tk.StringVar(value="A*")
        algo_box = ttk.Combobox(top, textvariable=self.algo, values=["UCS","A*"], state="readonly", width=6)
        algo_box.pack(side=tk.LEFT, padx=6)
        algo_box.bind("<<ComboboxSelected>>", lambda e: self._on_algo_change())

        tk.Label(top, text="Heuristic").pack(side=tk.LEFT, padx=(12,2))
        self.heur_name = tk.StringVar(value="Manhattan")
        self.heur_box = ttk.Combobox(top, textvariable=self.heur_name, values=list(HEURISTICS.keys()), state="readonly", width=12)
        self.heur_box.pack(side=tk.LEFT, padx=6)

        self.btn_new = tk.Button(top, text="New Grid", command=self.new_grid); self.btn_new.pack(side=tk.LEFT, padx=6)
        self.btn_run = tk.Button(top, text="Run Optimal", command=self.run_optimal); self.btn_run.pack(side=tk.LEFT, padx=6)
        self.btn_pause = tk.Button(top, text="Stop", command=self.pause); self.btn_pause.pack(side=tk.LEFT, padx=6)
        self.btn_resume = tk.Button(top, text="Continue", command=self.resume); self.btn_resume.pack(side=tk.LEFT, padx=6)
        self.btn_restart = tk.Button(top, text="Restart", command=self.restart); self.btn_restart.pack(side=tk.LEFT, padx=6)
        self.btn_stats = tk.Button(top, text="Stats", command=self.show_stats); self.btn_stats.pack(side=tk.LEFT, padx=6)

        self.info = tk.Label(self.root, text="", anchor="w"); self.info.pack(side=tk.TOP, fill=tk.X, padx=8)
        self.expanded_nodes = 0

        # Scrollable canvas
        container = tk.Frame(self.root); container.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.canvas = tk.Canvas(container, bg="white")
        xscroll = tk.Scrollbar(container, orient=tk.HORIZONTAL, command=self.canvas.xview)
        yscroll = tk.Scrollbar(container, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        container.rowconfigure(0, weight=1); container.columnconfigure(0, weight=1)

        # mousewheel scrolling
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Shift-MouseWheel>", self._on_shift_mousewheel)
        self.canvas.bind("<Button-4>", lambda e: self.canvas.yview_scroll(-1, "units"))
        self.canvas.bind("<Button-5>", lambda e: self.canvas.yview_scroll(+1, "units"))

        # world
        self.grid: Grid = None
        self.start: Pos = (0,0)
        self.green: Set[Pos] = set()
        self.red: Set[Pos] = set()
        self.robot: Pos = (0,0)
        self.trail: Set[Pos] = set()
        self.running: bool = False
        self.paused: bool = False
        self.collected_count: int = 0
        self.step_delay_ms: int = 30
        self.explore_current = None

        # dynamic red spawn config
        self.red_spawn_total = 0
        self.red_spawn_left = 0
        self.red_spawn_period_ms = 1000

        # Track scheduled after() jobs for robust restart
        self._after_jobs = set()

        # timer state
        self.start_time = None
        self.elapsed_sec = 0.0

        # track target-set signature to control replans
        self._last_targets_sig = None

        self.new_grid()
        self._on_algo_change()

    
    def _fixed_red_positions(self, count: int) -> list[Pos]:
        n = self.grid.width
        out, seen = [], set()
        a, c, m = 1103515245, 12345, 2**31
        seed = 2025
        def nxt():
            nonlocal seed
            seed = (a*seed + c) % m
            x = (seed // 131071) % n
            seed = (a*seed + c) % m
            y = (seed // 524287) % n
            return int(x), int(y)
        while len(out) < count:
            p = nxt()
            if p == self.start or p in seen: 
                continue
            seen.add(p); out.append(p)
        return out

    def _next_deterministic_free_cell(self, start_idx: int = 0):
        """Deterministic scan across the grid that always finds a free cell if one exists.
        Uses a stride relatively prime to n to cover all cells before repeating."""
        n = self.grid.width
        stride = 7 if n % 7 != 0 else 5  # small prime-ish stride co-prime with many n (works for n=10,20,50)
        total = n*n
        idx = start_idx % total
        for _ in range(total):
            x = (idx // n) % n
            y = (idx % n)
            p = (x, y)
            if p != self.start and p not in self.green and p not in self.red:
                return p
            idx = (idx + stride) % total
        return None
    def after(self, ms, cb):
            job = self.root.after(ms, cb)
            self._after_jobs.add(job)
            return job

    def _cancel_all_after(self):
        for job in list(self._after_jobs):
            try:
                self.root.after_cancel(job)
            except Exception:
                pass
        self._after_jobs.clear()

    def _start_timer(self):
        self.start_time = time.perf_counter() - self.elapsed_sec
        self.after(100, self._tick_timer)

    def _tick_timer(self):
        if not self.running or self.paused:
            return
        now = time.perf_counter()
        self.elapsed_sec = max(0.0, now - self.start_time)
        self.update_info()
        self.after(100, self._tick_timer)

    def _stop_timer(self):
        pass

    def _reset_timer(self):
        self.start_time = None
        self.elapsed_sec = 0.0

        # track target-set signature to control replans
        self._last_targets_sig = None
        self.update_info()

    def _on_mousewheel(self, event):
        delta = -1 if event.delta > 0 else 1
        self.canvas.yview_scroll(delta, "units")
    def _on_shift_mousewheel(self, event):
        delta = -1 if event.delta > 0 else 1
        self.canvas.xview_scroll(delta, "units")

    def new_grid(self):
        if self.running:
            return
        n = int(self.grid_size.get())
        rng = random.Random()
        self.start = (0,0)
        self.grid = Grid(n, n, walls=set())

        self.green = set()
        green_count = 4
        while len(self.green) < green_count:
            p = (rng.randrange(n), rng.randrange(n))
            if p != self.start:
                self.green.add(p)

        self.red = set()
        self.trail = set()
        self.robot = self.start
        self.running = False
        self.paused = False
        self._reset_timer()
        self.collected_count = 0
        self.update_info()
        self.draw_static(); self.draw_dynamic()

    
    def restart(self):
            self.running = False
            self.paused = False
            self._cancel_all_after()
            self.explore_current = None
            self.red.clear(); self.green.clear(); self.trail.clear()
            n = self.grid.width
            self._fixed_reds = getattr(self, "_fixed_reds", None) or self._fixed_red_positions(4)
            rng = random.Random(); green_count = 4
            while len(self.green) < green_count:
                p = (rng.randrange(n), rng.randrange(n))
                if p != self.start and p not in self._fixed_reds:
                    self.green.add(p)
            self.collected_count = 0; self.expanded_nodes = 0
            self.red_spawn_left = 0; self.red_spawn_total = 0
            self.robot = self.start; self._reset_timer()
            self.btn_run.config(state="normal"); self.btn_new.config(state="normal")
            self.draw_static(); self.draw_dynamic(); self.update_info()

    def bbox(self, p: Pos):
        x0 = PAD + p[0]*CELL; y0 = PAD + p[1]*CELL
        return (x0, y0, x0+CELL, y0+CELL)

    def draw_static(self):
        n = self.grid.width
        W = PAD*2 + n*CELL
        H = PAD*2 + n*CELL
        self.canvas.config(scrollregion=(0, 0, W, H))
        self.canvas.config(width=min(W, 900), height=min(H, 700))

        self.canvas.delete("all")
        for x in range(n):
            for y in range(n):
                x0,y0,x1,y1 = self.bbox((x,y))
                self.canvas.create_rectangle(x0,y0,x1,y1, fill="#ffffff", outline="#e5e7eb")
        sx0,sy0,sx1,sy1 = self.bbox(self.start)
        self.canvas.create_rectangle(sx0,sy0,sx1,sy1, outline="#2563eb", width=3)

    def draw_dynamic(self, frontier=None, explored=None, current=None):
        # overlays first
        if explored:
            for p in explored:
                x0,y0,x1,y1 = self.bbox(p)
                self.canvas.create_rectangle(x0,y0,x1,y1, fill="#bfdbfe", outline="")
        if frontier:
            for p in frontier:
                x0,y0,x1,y1 = self.bbox(p)
                self.canvas.create_rectangle(x0,y0,x1,y1, fill="#fcd34d", outline="")

        # trail above overlays
        for p in self.trail:
            x0,y0,x1,y1 = self.bbox(p); pad=8
            self.canvas.create_rectangle(x0+pad,y0+pad,x1-pad,y1-pad, fill="#93c5fd", width=0)

        # objects above trail
        for p in self.green:
            x0,y0,x1,y1 = self.bbox(p); pad=7
            self.canvas.create_oval(x0+pad,y0+pad,x1-pad,y1-pad, fill="#34d399", width=0)
        for p in self.red:
            x0,y0,x1,y1 = self.bbox(p); pad=7
            self.canvas.create_oval(x0+pad,y0+pad,x1-pad,y1-pad, fill="#ef4444", width=0)

        # current highlight
        if current is not None:
            x0,y0,x1,y1 = self.bbox(current)
            self.canvas.create_rectangle(x0+3,y0+3,x1-3,y1-3, outline="#ef4444", width=2)

        # robot on top
        x0,y0,x1,y1 = self.bbox(self.robot); pad=5
        self.canvas.create_rectangle(x0+pad,y0+pad,x1-pad,y1-pad, fill="#3b82f6", width=0)
        self.canvas.update()

    def pause(self):
        self.paused = True
        self._stop_timer()
    def resume(self):
        if self.paused:
            self.paused = False
            if self.running:
                self._start_timer()

    def _on_algo_change(self):
        if self.algo.get() == "UCS":
            self.heur_box.state(["disabled"])
        else:
            self.heur_box.state(["!disabled"])
        self.update_info()

    def update_info(self):
        t = fmt_time(self.elapsed_sec)
        self.info.config(text=f"Grid: {self.grid.width} | Algo: {self.algo.get()} | Heuristic: {self.heur_name.get()} | Collected: {self.collected_count} | Green left: {len(self.green)} | Red left: {len(self.red)} | Time: {t} | Expanded: {self.expanded_nodes}")

    def show_stats(self):
        messagebox.showinfo(
            "Run Stats",
            f"Grid size: {self.grid.width}\n"
            f"Algorithm: {self.algo.get()}\n"
            f"Collected so far: {self.collected_count}\n"
            f"Green remaining: {len(self.green)}\n"
            f"Red remaining: {len(self.red)}\n"
            f"Elapsed time: {fmt_time(self.elapsed_sec)}"
        )

    
    def _spawn_one_red(self):
            if not self.running or self.red_spawn_left <= 0:
                return
            n = self.grid.width
            if not hasattr(self, "_fixed_reds") or not self._fixed_reds:
                self._fixed_reds = self._fixed_red_positions(max(4, self.red_spawn_total or 4))

            # Try fixed list first
            base_idx = max(0, self.red_spawn_total - self.red_spawn_left)
            placed = False
            for k in range(len(self._fixed_reds)):
                candidate = self._fixed_reds[(base_idx + k) % len(self._fixed_reds)]
                x, y = candidate
                if 0 <= x < n and 0 <= y < n and candidate != self.start and candidate not in self.green and candidate not in self.red:
                    self.red.add(candidate)
                    self.red_spawn_left -= 1
                    placed = True
                    break

            # Fallback: deterministic grid scan that must find a slot if one exists
            if not placed:
                start_idx = base_idx * 17  # deterministic offset
                alt = self._next_deterministic_free_cell(start_idx)
                if alt is not None:
                    self.red.add(alt)
                    self.red_spawn_left -= 1
                    placed = True

            # Draw + schedule next spawn if any
            self.draw_static(); self.draw_dynamic(); self.update_info()
            if self.red_spawn_left > 0 and self.running:
                self.after(self.red_spawn_period_ms, self._spawn_one_red)

    def run_optimal(self):
        if self.running: return
        self.running = True
        self.paused = False
        self.explore_current = None
        self.expanded_nodes = 0
        self._start_timer()
        self.trail = set(); self.trail.add(self.robot)
        self.red.clear()
        self.red_spawn_total = 4
        self.red_spawn_left = self.red_spawn_total
        self._fixed_reds = getattr(self, "_fixed_reds", None) or self._fixed_red_positions(self.red_spawn_total)
        self._spawn_one_red()
        if self.red_spawn_left > 0 and self.running:
            self.after(self.red_spawn_period_ms, self._spawn_one_red)

        # capture signature of targets at plan time
        self._last_targets_sig = frozenset(self.green | self.red)
        self.btn_run.config(state="disabled"); self.btn_new.config(state="disabled")
        self._run_optimal_loop()

    def _run_optimal_loop(self):
        if not self.running:
            return
        points: List[Pos] = [self.start]
        current_idx = 0
        targets = list(self.green | self.red)
        if self.robot != self.start and self.robot not in targets:
            current_idx = len(points) + len(targets)
            points.extend(targets)
            points.append(self.robot)
        else:
            points.extend(targets)
            current_idx = 0 if self.robot == self.start else points.index(self.robot)

        algo = self.algo.get()
        h_map = {"Manhattan": h_manhattan, "Euclidean": h_euclidean, "Chebyshev": h_chebyshev}
        h_fn = h_map.get(self.heur_name.get(), h_manhattan)

        D, P = pairwise_distances(self.grid, points, algo=("ucs" if algo=="UCS" else "astar"), h=h_fn)
        if D is None:
            self.red_spawn_left = 0
            self.running=False
            self.btn_run.config(state="normal"); self.btn_new.config(state="normal")
            self._stop_timer()
            self.update_info()
            return

        order = optimal_order(points, D, home_idx=0, current_idx=current_idx)
        if len(order) < 2:
            self.running=False
            self.btn_run.config(state="normal"); self.btn_new.config(state="normal")
            self._stop_timer()
            self.update_info()
            return
        waypoints = [points[i] for i in order]
        # refresh signature for this plan
        self._last_targets_sig = frozenset(self.green | self.red)

        def run_segment(i):
            if not self.running:
                return
            if i >= len(waypoints)-1:
                self.running=False
                self.btn_run.config(state="normal"); self.btn_new.config(state="normal")
                self._stop_timer()
                self.update_info()
                return
            s = waypoints[i]; g = waypoints[i+1]
            gen = ucs_search(self.grid, s, g) if algo == "UCS" else astar_search(self.grid, s, g, h=h_fn)

            def step_explore():
                if not self.running:
                    return
                if self.paused:
                    self.after(50, step_explore); return
                try:
                    tag, payload = next(gen)
                    if tag == 'step':
                        self.explore_current = payload['current']
                        self.draw_static(); self.draw_dynamic(frontier=payload['frontier'], explored=payload['explored'], current=self.explore_current)
                        self.update_info()
                        self.expanded_nodes += 1
                    self.after(10, step_explore)
                except StopIteration as e:
                    path = e.value or []
                    self.explore_current = None

                    def animate_path(j=1):
                        if not self.running:
                            return
                        if j >= len(path):
                            # Only replan if targets actually changed since last plan
                            current_sig = frozenset(self.green | self.red)
                            if self._last_targets_sig != current_sig:
                                self._last_targets_sig = current_sig
                                self.after(10, self._run_optimal_loop)
                            else:
                                self.after(10, lambda: run_segment(i+1))
                            return
                        if self.paused:
                            self.after(50, lambda: animate_path(j)); return
                        self.robot = path[j]
                        self.trail.add(self.robot)
                        if self.robot in self.green: self.green.remove(self.robot); self.collected_count += 1
                        if self.robot in self.red: self.red.remove(self.robot); self.collected_count += 1
                        self.draw_static(); self.draw_dynamic()
                        self.update_info()
                        self.after(self.step_delay_ms, lambda: animate_path(j+1))
                    animate_path()

            step_explore()

        run_segment(0)

    def mainloop(self):
        self.root.mainloop()

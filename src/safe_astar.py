"""Safe A* reference implementation based on the method documented in ICCECT 2023."""
from dataclasses import dataclass
import heapq
import math

Cell = tuple[int, int]

@dataclass
class PlanResult:
    path: list[Cell] | None
    expanded_nodes: int

def _in_bounds(grid, c):
    x, y = c
    return 0 <= y < len(grid) and 0 <= x < len(grid[0])

def inflate_obstacles(grid, radius=1):
    h, w = len(grid), len(grid[0])
    out = [row[:] for row in grid]
    occupied = [(x, y) for y in range(h) for x in range(w) if grid[y][x]]
    for ox, oy in occupied:
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                c = (ox + dx, oy + dy)
                if _in_bounds(grid, c):
                    out[c[1]][c[0]] = 1
    return out

def _near_obstacle(grid, c):
    x, y = c
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            n = (x + dx, y + dy)
            if _in_bounds(grid, n) and grid[n[1]][n[0]]:
                return True
    return False

def _heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def _neighbors(grid, c):
    x, y = c
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
        n = (x + dx, y + dy)
        if not _in_bounds(grid, n) or grid[n[1]][n[0]]:
            continue
        if dx and dy:
            if grid[y][x + dx] or grid[y + dy][x]:
                continue
            step = math.sqrt(2.0)
        else:
            step = 1.0
        yield n, step

def _reconstruct(parent, goal):
    path = [goal]
    while path[-1] in parent:
        path.append(parent[path[-1]])
    path.reverse()
    return path

def plan(grid, start, goal, *, inflation_radius=0, adaptive_ga=False, reject_near_obstacles=False):
    work = inflate_obstacles(grid, inflation_radius) if inflation_radius else [r[:] for r in grid]
    if not _in_bounds(work, start) or not _in_bounds(work, goal):
        return PlanResult(None, 0)
    if work[start[1]][start[0]] or work[goal[1]][goal[0]]:
        return PlanResult(None, 0)

    # A practical public reimplementation of the paper's GA idea:
    # as the search proceeds through obstacle-free cells, accumulated g-cost
    # receives progressively less weight; encountering an obstacle-near region
    # resets that tendency. This preserves the documented direction of the method
    # without claiming to reproduce unavailable historical source code line-for-line.
    open_heap = [(0.0, 0.0, 1, start)]
    best_g = {start: 0.0}
    parent = {}
    closed = set()
    expanded = 0

    while open_heap:
        _, g, ga, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if current == goal:
            return PlanResult(_reconstruct(parent, goal), expanded)

        for nxt, step in _neighbors(work, current):
            near = _near_obstacle(work, nxt)
            if reject_near_obstacles and near:
                continue
            new_g = g + step
            if new_g >= best_g.get(nxt, math.inf):
                continue
            best_g[nxt] = new_g
            parent[nxt] = current
            next_ga = 1 if near else ga + 1
            g_weight = (1.0 / next_ga) if adaptive_ga else 1.0
            z = math.inf if (reject_near_obstacles and near) else 0.0
            f = g_weight * new_g + _heuristic(nxt, goal) + z
            heapq.heappush(open_heap, (f, new_g, next_ga, nxt))

    return PlanResult(None, expanded)

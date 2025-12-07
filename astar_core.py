from heapq import heappush, heappop
import time
import math
# core

def astar(grid, start_xy, goal_xy, heuristic_fn, deadline_ms=None):
    """
    grid: 2D list of 0 (free) / 1 (blocked)
    start_xy, goal_xy: (x, y)
    heuristic_fn: function (p, goal_xy) -> estimate cost
    deadline_ms: max time in milliseconds (None = no deadline)

    returns: (path_xy, hit_deadline)
        path_xy = list[(x, y)] from start to goal,
                   or best-so-far path if deadline hit
        hit_deadline = True if time limit exceeded
    """
    rows, cols = len(grid), len(grid[0])

    def in_bounds(p):
        x, y = p
        return 0 <= x < cols and 0 <= y < rows

    def passable(p):
        x, y = p
        return grid[y][x] == 0

    def neighbors(p):
        x, y = p
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:  # 4-dir
            nxt = (x + dx, y + dy)
            if in_bounds(nxt) and passable(nxt):
                yield nxt

    def h(p):
        return heuristic_fn(p, goal_xy)

    start_t = time.perf_counter()

    open_heap = []
    heappush(open_heap, (h(start_xy), 0, start_xy))

    came_from = {start_xy: None}
    g_score = {start_xy: 0.0}

    best_node = start_xy

    while open_heap:
        if deadline_ms is not None:
            elapsed_ms = (time.perf_counter() - start_t) * 1000.0
            if elapsed_ms > deadline_ms:
                path = reconstruct_path(came_from, best_node)
                return path, True

        f, g, current = heappop(open_heap)

        if h(current) < h(best_node):
            best_node = current

        if current == goal_xy:
            path = reconstruct_path(came_from, current)
            return path, False

        for nxt in neighbors(current):
            tentative_g = g_score[current] + 1.0  # each step cost 1
            if tentative_g < g_score.get(nxt, math.inf):
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                f_score = tentative_g + h(nxt)
                heappush(open_heap, (f_score, tentative_g, nxt))

    # No path exists
    return None, False


def reconstruct_path(came_from, node):
    if node is None or node not in came_from:
        return []
    path = []
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path
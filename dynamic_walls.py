import random
from typing import List, Optional, Set, Tuple

import world
from astar_core import astar


def manhattan(p, goal):
    return abs(p[0] - goal[0]) + abs(p[1] - goal[1])


def in_bounds(xy: Tuple[int, int]) -> bool:
    x, y = xy
    return 0 <= x < len(world.COLS) and 0 <= y < len(world.ROWS)


def neighbors4_labels(label: str) -> Set[str]:
    """Return the 4-neighbor labels of a given cell."""
    x, y = world.label_to_xy(label)
    out = set()
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = x + dx, y + dy
        if in_bounds((nx, ny)):
            out.add(world.xy_to_label(nx, ny))
    return out


def path_exists_unbounded(start_label: str, goal_label: str) -> bool:
    """Check if a path exists using A* with no deadline (guaranteed complete)."""
    start_xy = world.label_to_xy(start_label)
    goal_xy = world.label_to_xy(goal_label)
    path_xy, _ = astar(world.grid, start_xy, goal_xy, manhattan, deadline_ms=None)
    return path_xy is not None and len(path_xy) > 0


def choose_candidates(
    drone_label: str,
    current_path: Optional[List[str]],
    avoid: Set[str],
    ahead_window: int = 6,
) -> List[str]:
    """
    Build a list of candidate cells for wall placement.
    Prioritizes cells ahead on the current path, then random free cells.
    """
    candidates: List[str] = []

    # First priority: cells ahead on the current path
    if current_path:
        if drone_label in current_path:
            i = current_path.index(drone_label)
            ahead = current_path[i + 1 : i + 1 + ahead_window]
        else:
            ahead = current_path[:ahead_window]

        for lbl in ahead:
            if lbl in avoid:
                continue
            x, y = world.label_to_xy(lbl)
            if world.grid[y][x] == 0:
                candidates.append(lbl)

    # Second priority: random free cells
    all_labels = [
        world.xy_to_label(x, y)
        for y in range(len(world.ROWS))
        for x in range(len(world.COLS))
    ]
    random.shuffle(all_labels)

    for lbl in all_labels:
        if lbl in avoid:
            continue
        x, y = world.label_to_xy(lbl)
        if world.grid[y][x] == 0:
            candidates.append(lbl)

    # Deduplicate while preserving order
    seen = set()
    out = []
    for c in candidates:
        if c not in seen:
            out.append(c)
            seen.add(c)
    return out


def try_place_annoying_wall(
    drone_label: str,
    goal_label: str,
    current_path: Optional[List[str]],
    forbid_neighbors: bool = True,
    max_tries: int = 120,
) -> Optional[str]:
    """
    Try to place a wall that blocks the path but keeps goal reachable.
    
    - Never places on drone cell or goal cell
    - Optionally avoids drone's 4-neighbors
    - Verifies path still exists after placement (using unbounded A*)
    
    Returns the label where wall was placed, or None if no valid spot found.
    """
    avoid = {drone_label, goal_label}
    if forbid_neighbors:
        avoid |= neighbors4_labels(drone_label)

    candidates = choose_candidates(drone_label, current_path, avoid)

    tries = 0
    for lbl in candidates:
        if tries >= max_tries:
            break
        tries += 1

        # Temporarily place wall
        world.set_obstacle(lbl)

        # Check if path still exists
        if path_exists_unbounded(drone_label, goal_label):
            return lbl  # Wall placed successfully

        # Revert if it would block all paths
        world.clear_obstacle(lbl)

    return None

from world import grid, label_to_xy, xy_to_label
from astar_core import astar

# speedy
def manhattan(p, goal):
    return abs(p[0] - goal[0]) + abs(p[1] - goal[1])


def plan_v2(start_label: str, goal_label: str, deadline_ms: float):
    """
    A* with a time deadline in milliseconds.
    Returns (path_labels, hit_deadline)
    """
    start_xy = label_to_xy(start_label)
    goal_xy = label_to_xy(goal_label)

    path_xy, hit_deadline = astar(grid, start_xy, goal_xy, manhattan, deadline_ms=deadline_ms)

    if path_xy is None:
        return None, hit_deadline

    path_labels = [xy_to_label(x, y) for (x, y) in path_xy]
    return path_labels, hit_deadline

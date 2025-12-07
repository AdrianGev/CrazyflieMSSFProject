from world import grid, label_to_xy, xy_to_label
from astar_core import astar

# baseline
def manhattan(p, goal):
    return abs(p[0] - goal[0]) + abs(p[1] - goal[1])


def plan_v1(start_label: str, goal_label: str):
    """
    Classic A*, Manhattan heuristic, no deadline.
    Returns list of labels like ["A1", "A2", "B2", ...] or None if no path.
    """
    start_xy = label_to_xy(start_label)
    goal_xy = label_to_xy(goal_label)

    path_xy, hit_deadline = astar(grid, start_xy, goal_xy, manhattan, deadline_ms=None)
    if path_xy is None:
        return None

    return [xy_to_label(x, y) for (x, y) in path_xy]

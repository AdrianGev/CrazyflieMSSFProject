from world import grid, label_to_xy, xy_to_label
from astar_core import astar
from ml_planning.grid_world import GridWorld
from ml_planning.neural_ranker import NeuralRanker
import os

# load trained neural ranker if available
_ranker = None
_model_path = 'ml_planning/models/neural_ranker.pt'
if os.path.exists(_model_path):
    _ranker = NeuralRanker.create_from_file(_model_path)
    print(f"loaded neural ranker from {_model_path}")
else:
    print(f"neural ranker not found at {_model_path}, using manhattan distance")


def neural_heuristic(p, goal):
    # use neural ranker if loaded, otherwise fall back to manhattan distance
    if _ranker is None:
        return abs(p[0] - goal[0]) + abs(p[1] - goal[1])
    
    # convert grid to GridWorld format for neural ranker
    world = GridWorld(width=4, height=7)
    for y in range(7):
        for x in range(4):
            if grid[y][x] == 1:
                world.set_obstacle(x, y)
    
    # get neural score
    score = _ranker.score_node(p[0], p[1], goal[0], goal[1], world)
    
    # combine with manhattan distance (hybrid heuristic)
    base_h = abs(p[0] - goal[0]) + abs(p[1] - goal[1])
    return base_h + score


def plan_v3(start_label: str, goal_label: str, deadline_ms: float = None):
    start_xy = label_to_xy(start_label)
    goal_xy = label_to_xy(goal_label)

    path_xy, hit_deadline = astar(grid, start_xy, goal_xy, neural_heuristic, deadline_ms=deadline_ms)

    if path_xy is None:
        return None, hit_deadline

    path_labels = [xy_to_label(x, y) for (x, y) in path_xy]
    return path_labels, hit_deadline
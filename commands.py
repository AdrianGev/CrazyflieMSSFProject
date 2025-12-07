from world import label_to_xy
from crazyflie_control import fly_moves
# cmds

def path_labels_to_deltas(path_labels):
    """
    ["A1", "A2", "B2"] -> [(0, +1), (+1, 0)]
    """
    coords = [label_to_xy(lbl) for lbl in path_labels]
    deltas = []
    for (x1, y1), (x2, y2) in zip(coords, coords[1:]):
        dx = x2 - x1
        dy = y2 - y1
        deltas.append((dx, dy))
    return deltas


def deltas_to_move_names(deltas):
    """
    [(0, +1), (+1, 0)] -> ["down", "right"]
    (assuming y+ is 'down' on your board)
    """
    moves = []
    for dx, dy in deltas:
        if dx == 1 and dy == 0:
            moves.append("right")
        elif dx == -1 and dy == 0:
            moves.append("left")
        elif dx == 0 and dy == 1:
            moves.append("down")
        elif dx == 0 and dy == -1:
            moves.append("up")
        else:
            raise ValueError(f"Non-4-dir step: dx={dx}, dy={dy}")
    return moves


def execute_path_on_cf(path_labels):
    """
      path_labels -> deltas -> move names -> Crazyflie flight
    """
    if not path_labels or len(path_labels) < 2:
        print("Path too short or empty so like nothing to do lol")
        return

    deltas = path_labels_to_deltas(path_labels)
    moves = deltas_to_move_names(deltas)

    print("Moves to fly:", moves)
    fly_moves(moves)
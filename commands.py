from world import label_to_xy
from crazyflie_control import fly_moves, fly_segments
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
    
    Grid coordinate convention:
      x+ (dx=+1) = "right" (columns A->B->C->D)
      x- (dx=-1) = "left"
      y+ (dy=+1) = "down" (rows 1->2->3->...)
      y- (dy=-1) = "up"
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


def compress_moves(moves):
    """
    ["right","right","right","down","down"]
    -> [("right", 3), ("down", 2)]
    """
    if not moves:
        return []

    out = []
    cur = moves[0]
    count = 1

    for m in moves[1:]:
        if m == cur:
            count += 1
        else:
            out.append((cur, count))
            cur = m
            count = 1

    out.append((cur, count))
    return out


def execute_path_on_cf(path_labels, compress=False):
    """
    Full pipeline:
      path_labels -> deltas -> move names -> (optional compress) -> Crazyflie flight
    """
    if not path_labels or len(path_labels) < 2:
        print("Path too short or empty, nothing to do.")
        return

    deltas = path_labels_to_deltas(path_labels)
    moves = deltas_to_move_names(deltas)

    if compress:
        segments = compress_moves(moves)
        print(f"Raw moves: {len(moves)} | Segments: {len(segments)}")
        print("Segments:", segments)
        fly_segments(segments)
        return

    print("Moves to fly:", moves)
    fly_moves(moves)
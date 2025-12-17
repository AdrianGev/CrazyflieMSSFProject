import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crazyflie.log import LogConfig
# ctrl
URI = 'radio://0/80/2M'

CELL = 0.10


def apply_move_to_xy(cur_x, cur_y, move, count=1):
    """Apply a move to current position using the same mapping as fly_moves/fly_segments."""
    if move == "right":
        cur_y -= CELL * count
    elif move == "left":
        cur_y += CELL * count
    elif move == "down":
        cur_x -= CELL * count
    elif move == "up":
        cur_x += CELL * count
    return cur_x, cur_y


def start_state_logger(cf, on_state):
    """
    Calls on_state(x, y, z) at ~10 Hz.
    """
    lg = LogConfig(name="state", period_in_ms=100)  # 10 Hz
    lg.add_variable("stateEstimate.x", "float")
    lg.add_variable("stateEstimate.y", "float")
    lg.add_variable("stateEstimate.z", "float")

    def _cb(timestamp, data, logconf):
        on_state(
            data["stateEstimate.x"],
            data["stateEstimate.y"],
            data["stateEstimate.z"],
        )

    cf.log.add_config(lg)
    lg.data_received_cb.add_callback(_cb)
    lg.start()
    return lg


def reset_estimator(cf: Crazyflie):
    """Reset Kalman estimator so it doesn't start with a weird offset."""
    print("Resetting estimator...")
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2.0)  # let it chill


def setup_cf(on_state=None):
    """Connect, set estimator, reset, return (cf, hl, target_z, logger)."""
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    cf.open_link(URI)

    time.sleep(1.0)

    cf.param.set_value('stabilizer.estimator', '2')
    time.sleep(0.1)

    reset_estimator(cf)

    hl = HighLevelCommander(cf)
    target_z = 0.25  # meters

    print("Takeoff")
    hl.takeoff(target_z, 1.5)
    time.sleep(2.0)

    print("Stabilizing at origin")
    for _ in range(10):
        hl.go_to(0.0, 0.0, target_z, 0.0, 0.20)
        time.sleep(0.25)

    logger = None
    if on_state is not None:
        logger = start_state_logger(cf, on_state)

    return cf, hl, target_z, logger


def teardown_cf(cf: Crazyflie, hl: HighLevelCommander, target_z: float, logger=None):
    """Land and close link."""
    if logger is not None:
        try:
            logger.stop()
        except Exception:
            pass

    print("Landing")
    hl.land(0.0, 1.5)
    time.sleep(2.0)

    cf.close_link()
    print("Link closed")


def fly_moves(moves, on_state=None):
    """
    moves: list like ["right", "right", "down", "left", ...]
    Each move = one CELL in world coordinates.

    Board-to-crazyflie coordinate mapping:
      Board "down" (rows 1->2->3) = crazyflie X+ (forward)
      Board "up"   (rows 3->2->1) = crazyflie X- (backward)
      Board "right" (cols A->B->C) = crazyflie Y- (right)
      Board "left"  (cols C->B->A) = crazyflie Y+ (left)
    """
    cf, hl, target_z, logger = setup_cf(on_state=on_state)

    # local idea of where the drone is in (x, y) for crazyflie frame
    cur_x = 0.0
    cur_y = 0.0

    try:
        for m in moves:
            if m == "right":
                cur_y -= CELL  # crazyflie Y- = board right
            elif m == "left":
                cur_y += CELL  # crazyflie Y+ = board left
            elif m == "down":
                cur_x -= CELL  # crazyflie X- = board down
            elif m == "up":
                cur_x += CELL  # crazyflie X+ = board up
            else:
                print(f"Ignoring unknown jawn: {m}")
                continue

            print(f"Move {m} -> go_to({cur_x:.3f}, {cur_y:.3f}, {target_z:.3f})")
            # go_to(x, y, z, yaw, duration)
            hl.go_to(cur_x, cur_y, target_z, 0.0, 2.0)
            time.sleep(2.3)  # a bit longer than duration for safety

    finally:
        teardown_cf(cf, hl, target_z, logger=logger)


def fly_segments(segments, on_state=None):
    """
    segments: [("right", 3), ("down", 2), ...]
    Does one go_to per segment using the SAME mapping as fly_moves().
    """
    cf, hl, target_z, logger = setup_cf(on_state=on_state)

    cur_x = 0.0
    cur_y = 0.0

    try:
        for move, count in segments:
            if move == "right":
                cur_y -= CELL * count
            elif move == "left":
                cur_y += CELL * count
            elif move == "down":
                cur_x -= CELL * count
            elif move == "up":
                cur_x += CELL * count
            else:
                print(f"Ignoring unknown move: {move}")
                continue

            duration = 2.0 * count  # keep speed consistent with fly_moves()
            print(
                f"Segment {move} x{count} -> go_to({cur_x:.3f}, {cur_y:.3f}, {target_z:.3f})"
            )
            hl.go_to(cur_x, cur_y, target_z, 0.0, duration)
            time.sleep(duration + 0.3)

    finally:
        teardown_cf(cf, hl, target_z, logger=logger)


def fly_replanning(step_provider, on_state=None):
    """
    Replanning flight: calls step_provider() for each move.
    step_provider returns a move string ("right", "left", etc.) or None to stop.
    """
    cf, hl, target_z, logger = setup_cf(on_state=on_state)
    cur_x, cur_y = 0.0, 0.0

    # Give logger time to send first state estimate
    time.sleep(0.5)

    try:
        while True:
            m = step_provider()
            if m is None:
                break

            cur_x, cur_y = apply_move_to_xy(cur_x, cur_y, m, count=1)
            print(f"[REPLAN] Move {m} -> go_to({cur_x:.3f}, {cur_y:.3f}, {target_z:.3f})")
            hl.go_to(cur_x, cur_y, target_z, 0.0, 2.0)
            time.sleep(2.3)

    finally:
        teardown_cf(cf, hl, target_z, logger=logger)


def fly_fixed_path_with_checks(path_labels, stop_if_blocked=True, on_state=None):
    """
    Follows a fixed planned path, but BEFORE each step checks if the NEXT cell is now blocked.
    If blocked: land in place.
    """
    import world

    cf, hl, target_z, logger = setup_cf(on_state=on_state)
    cur_x, cur_y = 0.0, 0.0

    try:
        for i in range(len(path_labels) - 1):
            nxt_lbl = path_labels[i + 1]

            x, y = world.label_to_xy(nxt_lbl)
            if stop_if_blocked and world.grid[y][x] == 1:
                print(f"[V1] Next cell {nxt_lbl} blocked -> landing now")
                break

            x1, y1 = world.label_to_xy(path_labels[i])
            x2, y2 = world.label_to_xy(nxt_lbl)
            dx, dy = x2 - x1, y2 - y1

            if dx == 1 and dy == 0:
                m = "right"
            elif dx == -1 and dy == 0:
                m = "left"
            elif dx == 0 and dy == 1:
                m = "down"
            elif dx == 0 and dy == -1:
                m = "up"
            else:
                print(f"[V1] Non-4dir step {path_labels[i]}->{nxt_lbl} -> landing")
                break

            cur_x, cur_y = apply_move_to_xy(cur_x, cur_y, m, count=1)
            hl.go_to(cur_x, cur_y, target_z, 0.0, 2.0)
            time.sleep(2.3)

    finally:
        teardown_cf(cf, hl, target_z, logger=logger)

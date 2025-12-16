import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
# ctrl
URI = 'radio://0/80/2M'

CELL = 0.10

def reset_estimator(cf: Crazyflie):
    """Reset Kalman estimator so it doesn't start with a weird offset."""
    print("Resetting estimator...")
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2.0)  # let it chill


def setup_cf():
    """Connect, set estimator, reset, return (cf, hl, target_z)."""
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

    return cf, hl, target_z


def teardown_cf(cf: Crazyflie, hl: HighLevelCommander, target_z: float):
    """Land and close link."""
    print("Landing")
    hl.land(0.0, 1.5)
    time.sleep(2.0)

    cf.close_link()
    print("Link closed")


def fly_moves(moves):
    """
    moves: list like ["right", "right", "down", "left", ...]
    Each move = one CELL in world coordinates.

    Board-to-crazyflie coordinate mapping:
      Board "down" (rows 1->2->3) = crazyflie X+ (forward)
      Board "up"   (rows 3->2->1) = crazyflie X- (backward)
      Board "right" (cols A->B->C) = crazyflie Y- (right)
      Board "left"  (cols C->B->A) = crazyflie Y+ (left)
    """
    cf, hl, target_z = setup_cf()

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
        teardown_cf(cf, hl, target_z)


def fly_segments(segments):
    """
    segments: [("right", 3), ("down", 2), ...]
    Does one go_to per segment using the SAME mapping as fly_moves().
    """
    cf, hl, target_z = setup_cf()

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
        teardown_cf(cf, hl, target_z)

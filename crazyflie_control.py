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
    hl.go_to(0.0, 0.0, target_z, 0.0, 2.0)
    time.sleep(2.3)

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

    Coordinate convention (world frame):
      x+ = "right" on your board
      x- = "left"
      y+ = "down"
      y- = "up"
    """
    cf, hl, target_z = setup_cf()

    # local idea of where the drone is in (x, y)
    cur_x = 0.0
    cur_y = 0.0

    try:
        for m in moves:
            if m == "right":
                cur_x += CELL
            elif m == "left":
                cur_x -= CELL
            elif m == "down":
                cur_y += CELL
            elif m == "up":
                cur_y -= CELL
            else:
                print(f"Ignoring unknown move: {m}")
                continue

            print(f"Move {m} -> go_to({cur_x:.3f}, {cur_y:.3f}, {target_z:.3f})")
            # go_to(x, y, z, yaw, duration)
            hl.go_to(cur_x, cur_y, target_z, 0.0, 2.0)
            time.sleep(2.3)  # a bit longer than duration for safety

    finally:
        teardown_cf(cf, hl, target_z)

"""Cell layout shared by the world generator, the PLC and both SCARA controllers.

World frame, metres, z up. Plan view (x to the right, y up):

        magazine A          magazine B
           tray A             tray B
        SCARA A (west)    SCARA B (east)
                 \\   pick   /
                  conveyor, parts travel +y
                  feeder (south)

Both robots are Epson T6-602S. They share one pick point and never enter it together.
Each one fills its own tray (6 pockets). A full tray leaves north, through the fence.
"""
import math

# ------------------------------------------------------------------ table and robots
TABLE = (-0.95, 0.95, -0.48, 0.70)          # x0, x1, y0, y1
TABLE_Z = 0.75
SCARA_RISER = 0.05
BASE_Z = TABLE_Z + SCARA_RISER
# yaw 0: arm at q = 0 points to +x. B is turned 180 deg so its q = 0 points to -x, toward the centre.
BASE = {
    "A": (-0.46, 0.00, BASE_Z, 0.0),
    "B": (0.46, 0.00, BASE_Z, math.pi),
}
SCARA_TOOL = 0.06                            # hand slot -> suction cup lip
CLEAR_Z = 0.90                               # TCP transfer height
HALF_X = 0.02                                # A stays at x <= this, B at x >= -this (elbow and TCP)
COLUMN_X = 0.10                              # |x| < COLUMN_X is the shared pick column

# ------------------------------------------------------------------ parts and trays
PART_R, PART_H = 0.018, 0.040
TRAY_L, TRAY_W, TRAY_H, TRAY_FLOOR = 0.200, 0.140, 0.045, 0.006
POCKETS = [(-0.055, -0.032), (0.0, -0.032), (0.055, -0.032),
           (-0.055, 0.032), (0.0, 0.032), (0.055, 0.032)]
PARTS_PER_TRAY = len(POCKETS)
PLACE_Z = TABLE_Z + TRAY_FLOOR + PART_H      # TCP on the part, sitting in a pocket

STATION = {"A": (-0.22, 0.34), "B": (0.22, 0.34)}
VIA = {"A": (-0.18, 0.00), "B": (0.18, 0.00)}     # outside the column, on the way home
MAG = {"A": (-0.78, 0.34), "B": (0.78, 0.34)}
MAG_COUNT = 4
OUT = {"A": (-0.22, 1.40), "B": (0.22, 1.40)}     # full tray, north of the fence
N_PARTS = 48

# ------------------------------------------------------------------ infeed (belt along +y, stop at the pick)
CONV_X = 0.0
CONV_Y0, CONV_Y1 = -1.55, -0.12
CONV_Z = 0.78                                # belt top
CONV_W = 0.10
BELT_SPEED = 0.22
PART_GAP = 0.050
PICK = (CONV_X, -0.16, CONV_Z + PART_H)      # TCP on the part waiting at the stop
SPAWN_Y = -1.32
ACCU_Y = PICK[1] - 4 * PART_GAP
FEED_PERIOD = 1.05

# ------------------------------------------------------------------ safety
FENCE = (-1.20, 1.20, -0.78, 1.02)           # x0, x1, y0, y1
DOOR = (1.20, 0.15, 0.70)                    # east wall: x, y centre, width
CONV_OPEN = (-0.14, 0.14)                    # south wall opening for the conveyor
OUT_OPEN = {"A": (-0.40, -0.04), "B": (0.04, 0.40)}
SCANNER = (0.0, 1.02)                        # on the north fence, looking north
WARN_R, STOP_R = 1.40, 0.75
SLOW = 0.40
SCAN_CLEAR_T = 1.5
OPERATOR_HOME = (1.85, 1.70)
OPERATOR_PATH = [(1.85, 1.70), (0.85, 1.55), (0.0, 1.45), (0.85, 1.55), (1.85, 1.70)]


def tray_def(side, index):
    return "TRAY_%s_%d" % (side, index)


def home_world(side):
    x, y = STATION[side]
    return (x, y, CLEAR_Z)


def via_world(side):
    x, y = VIA[side]
    return (x, y, CLEAR_Z)


def pocket_world(side, index):
    dx, dy = POCKETS[index]
    sx, sy = STATION[side]
    return (sx + dx, sy + dy, PLACE_Z)

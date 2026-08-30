"""Physical constants of the e-puck and its arena.

Every value here is read off the Webots R2025a PROTO sources
(`E-puck.proto`, `E-puckDistanceSensor.proto`, `RectangleArena.proto`)
"""

# ---- Geometry and motor limits ----

WHEEL_RADIUS_M: float = 0.0205
# Effective track width, calibrated against the simulator's ground truth. The
# geometric axle length (0.052 m) under-estimates rotation by roughly 10%
# because the wheels scrub against the floor as the robot turns.
AXLE_LENGTH_M: float = 0.057
MAX_WHEEL_SPEED: float = 6.28  # rad/s, E-puck.proto maxVelocity
BODY_RADIUS_M: float = 0.037

# ---- Arena (RectangleArena, floorSize 1x1; walls sit outside the floor) ----

ARENA_HALF_M: float = 0.5

# ---- Infra-red proximity sensors ----

PS_NAMES: tuple[str, ...] = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")

# E-puckDistanceSensor.proto lookupTable: distance in metres against the raw
# value the sensor reports. Raw falls off monotonically with distance.
LOOKUP_DISTANCE_M: tuple[float, ...] = (
    0.0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07,
)
LOOKUP_RAW: tuple[float, ...] = (
    4095.0, 2133.33, 1465.73, 601.46, 383.84, 234.93, 158.03, 120.0, 104.09, 67.19,
)

IR_MAX_RANGE_M: float = LOOKUP_DISTANCE_M[-1]

# Hits farther than this are used for obstacle avoidance but never written to
# the map: the lookupTable's noise column gives about 4% relative error at
# 0.05 m and 5% at 0.07 m, which smears obstacles across neighbouring cells.
IR_MAP_TRUST_RANGE_M: float = 0.05

# Mounting of ps0..ps7 in the robot frame (x forward, y left), from E-puck.proto.
SENSOR_OFFSETS_M: tuple[tuple[float, float], ...] = (
    (0.030, -0.010),   # ps0  front, 17 deg right
    (0.022, -0.025),   # ps1  46 deg right
    (0.000, -0.031),   # ps2  90 deg right
    (-0.030, -0.015),  # ps3  151 deg right
    (-0.030, 0.015),   # ps4  151 deg left
    (0.000, 0.031),    # ps5  90 deg left
    (0.022, 0.025),    # ps6  46 deg left
    (0.030, 0.010),    # ps7  front, 17 deg left
)
SENSOR_ANGLES_RAD: tuple[float, ...] = (
    -0.30, -0.80, -1.57, -2.64, 2.64, 1.57, 0.80, 0.30,
)

# The four sensors looking into the forward hemisphere.
FRONT_SENSORS: tuple[int, ...] = (0, 1, 6, 7)

# The forward-left and forward-right pairs, used to decide which way to turn
# when the robot has to back out of something.
LEFT_SENSORS: tuple[int, ...] = (5, 6)
RIGHT_SENSORS: tuple[int, ...] = (1, 2)

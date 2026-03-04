"""Shared constants for the Imperial Robotics Labs codebase.

All distances are in the unit noted per-section.
Every module should import constants from here rather than defining its own.
"""

import brickpi3

# -- Hardware ------------------------------------------------------------------
BP = brickpi3.BrickPi3()
BP_SENSOR_ERROR = brickpi3.SensorError

# Motor ports                          # used by: motion, planning, navigation
LEFT_MOTOR_PORT = BP.PORT_B
RIGHT_MOTOR_PORT = BP.PORT_C

# Sensor ports                         # used by: planning (commented-out bump)
LEFT_TOUCH_PORT = BP.PORT_1
RIGHT_TOUCH_PORT = BP.PORT_2

# -- Wheel geometry (mm) ------------------------------------------------------
PI = 3.14159627                         # used by: motion
WHEEL_DIAMETER = 67                     # used by: motion  (mm)
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI  # used by: motion, planning  (mm)
WHEELBASE_WIDTH = 152                   # used by: motion, planning  (mm)

# Wheel geometry (cm) - derived         # used by: planning
WHEEL_CIRCUMFERENCE_CM = WHEEL_CIRCUMFERENCE / 10.0
WHEELBASE_CM = WHEELBASE_WIDTH / 10.0

# -- Motion calibration -------------------------------------------------------
DISTANCE_ERROR = -1.15 * PI             # used by: motion  (mm)
ANGLE_ERROR = -3.074                    # used by: motion  (degrees)
MOVEMENT_SPEED = 400                    # used by: motion, navigation  (DPS)
TURNING_SPEED = 300                     # used by: motion  (DPS)
MINI_WAIT_TIME = 0.15                   # used by: motion  (seconds)
POSITION_TOLERANCE = 5                  # used by: motion  (encoder degrees)
TIMEOUT = 60                            # used by: motion  (seconds)

# -- DWA / Planning ------------------------------------------------------------
ROBOTRADIUS = 10.0                      # used by: planning  (cm)
B_RADIUS = 3.3                     # used by: planning  (cm)
SAFEDIST = 8.0                         # used by: planning  (cm)
MAXVELOCITY = 20.0                      # used by: planning  (cm/s)
MAXACCELERATION = 50.0                  # used by: planning  (cm/s^2)
GOAL_TOLERANCE = 5.0                    # used by: planning  (cm)
MAX_BARRIERS = 20                       # used by: planning
FORWARDWEIGHT = 12                      # used by: planning
OBSTACLEWEIGHT = 50                     # used by: planning
SPEEDWEIGHT = 2                         # used by: planning
HEADINGWEIGHT = 1000                      # used by: planning
TAU = 1.5                               # used by: planning  (seconds)
# DEDUP_RADIUS = BARRIERRADIUS + 10.0      # used by: planning  (cm)
Y_UNCERTAINTY = 30
X_UNCERTAINTY = 5
CAM_DIST = 90

# -- Particle filter -----------------------------------------------------------
NUM_PARTICLES = 100                     # used by: particleDataStructures
ROBOT_START_POS = (84, 30, 0, 1/NUM_PARTICLES)  # used by: particleDataStructures

E_MEAN, E_VAR = 0, 10                  # used by: particleDataStructures  (cm)
F_MEAN, F_VAR = 0, 1                   # used by: particleDataStructures  (degrees)
G_MEAN, G_VAR = 0, 1                   # used by: particleDataStructures  (degrees)
SONAR_VAR = 4                           # used by: particleDataStructures  (cm)
BASELINE_PROB = 0.01                    # used by: particleDataStructures

# -- Map geometry --------------------------------------------------------------
WAYPOINTS = [                           # used by: particleDataStructures, navigation
    (180, 30), (180, 54), (138, 54), (138, 168),
    (114, 168), (114, 84), (84, 84), (84, 30),
]

WALLS = [                               # used by: particleDataStructures, navigation
    (0,0,0,168), (0,168,84,168), (84,126,84,210), (84,210,168,210),
    (168,210,168,84), (168,84,210,84), (210,84,210,0), (210,0,0,0),
]

# -- Navigation ----------------------------------------------------------------
INTERVAL = 20                           # used by: navigation  (cm)
WAYPOINT_TOLERANCE = 1                  # used by: navigation  (cm)

# -- Sonar calibration ---------------------------------------------------------
SCALE_CALIBRATED = 1.0127               # used by: sonarSensor
OFFSET_CALIBRATED = 3.7975              # used by: sonarSensor  (cm)

# -- Vision --------------------------------------------------------------------
WHITE = (255, 255, 255)                 # used by: picameracans, planning
GREEN = (0, 255, 0)                     # used by: picameracans

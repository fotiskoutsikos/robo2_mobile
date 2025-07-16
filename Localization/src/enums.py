from enum import Enum

class state(Enum):
    px = 0
    py = 1
    theta = 2
    px_prev = 3
    py_prev = 4

class measurement(Enum):
    sonarF = 0
    sonarFL = 1
    sonarFR = 2
    sonarL = 3
    sonarR = 4
    yaw = 5

class input(Enum):
    acc_x = 0
    acc_y = 1
    ang_vel = 2

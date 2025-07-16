import numpy as np
import enums

class sonar_model:
    def __init__(self):
        L1 = 0.018
        L2 = 0.05
        L3 = 0.1
        L4 = 0.2

        self.max_distance = 2.0
        
        self.angles = np.zeros(enums.measurement.yaw.value)
        self.angles[enums.measurement.sonarF.value] = 0.0
        self.angles[enums.measurement.sonarFL.value] = np.pi / 4
        self.angles[enums.measurement.sonarFR.value] = - np.pi / 4
        self.angles[enums.measurement.sonarL.value] = np.pi / 2
        self.angles[enums.measurement.sonarR.value] = - np.pi / 2

        self.offset = [np.zeros(2) for _ in range(enums.measurement.yaw.value)]
        self.offset[enums.measurement.sonarF.value] = np.array([L4, 0])
        self.offset[enums.measurement.sonarFL.value] = np.array([L4-L1, L3-L1])
        self.offset[enums.measurement.sonarFR.value] = np.array([L4-L1, -(L3-L1)])
        self.offset[enums.measurement.sonarL.value] = np.array([0, L3])
        self.offset[enums.measurement.sonarR.value] = np.array([0, -L3])
        
        pass
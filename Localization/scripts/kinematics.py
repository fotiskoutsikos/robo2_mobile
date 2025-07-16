import numpy as np

class mobibot_kinematics:
    def __init__(self):
        self.wheel_diameter = 0.2
        # wheel to wheel distance
        self.l4 = 0.2
        # height of cylindrical wheel
        self.l2 = 0.05

        self.r = self.wheel_diameter / 2
        self.l = (self.l4 + self.l2) / 2

        # forward kinematics matrix
        # for getting heading speed and rotational speed from wheels' rotational speeds
        self.forward_mat = np.array([
            [self.r / 2, self.r / 2],
            [self.r / (2*self.l), -self.r / (2*self.l)]
        ])

        # inverse kinematics matrix
        self.inverse_mat = np.linalg.inv(self.forward_mat)

    
    def forward(self, wheel_speeds):
        return self.forward_mat @ wheel_speeds
    
    def inverse(self, robot_speeds):
        return self.inverse_mat @ robot_speeds

    
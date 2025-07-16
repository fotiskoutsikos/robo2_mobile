#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""
from gazebo_msgs.msg import ModelStates
# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

import logger
import ekf
import enums

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_localization():
    """Class to compute and publish joints positions"""
    def __init__(self):
        rospy.on_shutdown(self.turn_off)

        self.logger = logger.SimulationLogger()
        self.rate = 2 #Hz
        mean = np.zeros(len(enums.state))
        mean[enums.state.theta.value] = 1.8584
        covariance_matrix = 0.1 * np.eye(len(enums.state))
        self.filter = ekf.ekf(self.rate, mean, covariance_matrix)
        
        # joints' states
        self.joint_states = JointState()

        # Sensors
        self.clean_data = {
            "imu": {
                "yaw": 1.8584,
                "angular_velocity_z": 0.0,
                "acceleration_x": 0.0,
                "acceleration_y": 0.0
            },
            "sonar": {
                "F": 2.0, "FL": 2.0, "FR": 2.0, "L": 2.0, "R": 2.0
            }
        }

        self.noisy_data = {
            "imu": {
                "yaw": 0.0,
                "angular_velocity_z": 0.0,
                "acceleration_x": 0.0,
                "acceleration_y": 0.0
            },
            "sonar": {
                "F": 0.0, "FL": 0.0, "FR": 0.0, "L": 0.0, "R": 0.0
            }
        }


        self.noise_std = {
            "sonar": 0.01,#m
            "acceleration": 0.002,#m/s^2
            "angular_velocity": 0.002,#rad/s
            "angle": 0.002#rad
        }

        self.bot_pose = {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0
        }
        # Seed for reproducibility
        np.random.seed(42)

        # ROS SETUP
        # initialize subscribers for reading encoders
        # Robot
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # getting bot's pose to benchmark the localization algorithm
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        self.locate()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])


    def model_states_callback(self, msg):
        try:
            idx = msg.name.index("mymobibot")
            pose = msg.pose[idx]

            # Extract position
            x = pose.position.x
            y = pose.position.y

            # Extract yaw from orientation (convert quaternion to euler)
            orientation_q = pose.orientation
            (_, _, yaw) = quaternion_to_euler(
                orientation_q.w,
                orientation_q.x,
                orientation_q.y,
                orientation_q.z
            )

            # Store as a simple dictionary
            self.bot_pose = {
                "x": x,
                "y": y,
                "yaw": yaw
            }

        except ValueError:
            rospy.logwarn("mymobibot not found in /gazebo/model_states")

    
    def imu_callback(self, msg):
        # ROS callback to get the /imu

        (roll, pitch, yaw) = quaternion_to_euler(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )

        self.clean_data["imu"]["yaw"] = yaw
        self.clean_data["imu"]["angular_velocity_z"] = msg.angular_velocity.z
        self.clean_data["imu"]["acceleration_x"] = msg.linear_acceleration.x
        self.clean_data["imu"]["acceleration_y"] = msg.linear_acceleration.y

        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)

    def sonar_front_callback(self, msg):
        self.clean_data["sonar"]["F"] = msg.range

    def sonar_frontleft_callback(self, msg):
        self.clean_data["sonar"]["FL"] = msg.range

    def sonar_frontright_callback(self, msg):
        self.clean_data["sonar"]["FR"] = msg.range

    def sonar_left_callback(self, msg):
        self.clean_data["sonar"]["L"] = msg.range

    def sonar_right_callback(self, msg):
        self.clean_data["sonar"]["R"] = msg.range
        
    def update_noisy_data(self):
        # --- Update noisy sonar readings ---
        sonar_std = self.noise_std["sonar"]
        for key, val in self.clean_data["sonar"].items():
            if val != float('inf'):
                noisy_val = val + np.random.normal(0, sonar_std)
                self.noisy_data["sonar"][key] = max(0.0, noisy_val)  # sonar can't be negative
            else:
                self.noisy_data["sonar"][key] = float('inf')

        # --- Update noisy IMU readings ---
        imu_msg = self.clean_data["imu"]
        if imu_msg is not None:
            accel_x = self.clean_data["imu"]["acceleration_x"]
            accel_y = self.clean_data["imu"]["acceleration_y"]
            ang_vel_z = self.clean_data["imu"]["angular_velocity_z"]
            yaw = self.clean_data["imu"]["yaw"]
            
            accel_noisy_x = accel_x + np.random.normal(0, self.noise_std["acceleration"])
            accel_noisy_y = accel_y + np.random.normal(0, self.noise_std["acceleration"])
            ang_vel_z_noisy = ang_vel_z + np.random.normal(0, self.noise_std["angular_velocity"])
            yaw_noisy = yaw + np.random.normal(0, self.noise_std["angle"])

            self.noisy_data["imu"]["acceleration_x"] = accel_noisy_x
            self.noisy_data["imu"]["acceleration_y"] = accel_noisy_y
            self.noisy_data["imu"]["yaw"] = yaw_noisy
            self.noisy_data["imu"]["angular_velocity_z"] = ang_vel_z_noisy

    def locate(self):
        rate = rospy.Rate(self.rate)
        print("The system is ready to start the localization algorithm...")

        last_time = rospy.get_time()
        rate.sleep()

        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            dt = current_time - last_time
            last_time = current_time

            # 1. Get sensor measurements
            self.update_noisy_data()
            
            # 2. Predict + Update (run EKF here, when you add it)
            # read from noisy data
            # self.filter.read_data(self.noisy_data)
            # est_state, est_covariance = self.filter.estimate()
            # 3. Log data
            self.logger.log_pose(self.bot_pose)
            self.logger.log_time(current_time)
            self.logger.log_sonar(self.noisy_data["sonar"])
            self.logger.log_imu(self.noisy_data["imu"])

            #self.logger.log_ekf_state(est_state)
            #self.logger.log_ekf_covariance(est_covariance)

            rate.sleep()
            
    def turn_off(self):
        print("logging data")
        self.logger.save_to_csv()

def localization_py():
    # Starts a new node
    rospy.init_node('localization_node', anonymous=True)
    
    localizer = mymobibot_localization()
    # rospy.on_shutdown(localization.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        localization_py()
    except rospy.ROSInterruptException:
        pass

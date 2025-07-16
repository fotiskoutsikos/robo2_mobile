import numpy as np
import os
import pandas as pd
import logger
import ekf
import enums
import plotter

class offline_simulator:
    def __init__(self, logs_dir):

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

        self.bot_pose = {
                "x": 0,
                "y": 0,
                "yaw": 0
            }
        
        # Logs and plots directory setup
        self.logs_dir = logs_dir
        
        # Load CSV files
        time_data = pd.read_csv(os.path.join(self.logs_dir, 'time.csv'))
        sonar_data = pd.read_csv(os.path.join(self.logs_dir, 'sonar_measurements.csv'))
        imu_data = pd.read_csv(os.path.join(self.logs_dir, 'imu_measurements.csv'))
        pose_data = pd.read_csv(os.path.join(self.logs_dir, 'pose.csv'))

        # Assign to instance variables
        self.time = time_data['time']
        self.sonar_data = sonar_data[['F', 'FL', 'FR', 'L', 'R']].values
        self.imu_data = imu_data
        self.pose_data = pose_data[['x', 'y', 'yaw']].values

        self.rate = 1 / ((self.time[len(self.time)-1] - self.time[0]) / len(self.time))        
        self.logger = logger.SimulationLogger('src//logs')
        
        mean = np.zeros(len(enums.state))
        covariance_matrix = 0.1 * np.eye(len(enums.state))
        self.filter = ekf.ekf(self.rate, mean, covariance_matrix)
    
    def update_data(self, index):
        self.noisy_data = {
            "imu": {
                "yaw": self.imu_data['yaw'][index],
                "angular_velocity_z": self.imu_data['angular_velocity_z'][index],
                "acceleration_x": self.imu_data['acceleration_x'][index],
                "acceleration_y": self.imu_data['acceleration_y'][index]
            },
            "sonar": {
                "F": self.sonar_data[index][enums.measurement.sonarF.value],
                "FL": self.sonar_data[index][enums.measurement.sonarFL.value],
                "FR": self.sonar_data[index][enums.measurement.sonarFR.value],
                "L": self.sonar_data[index][enums.measurement.sonarL.value],
                "R": self.sonar_data[index][enums.measurement.sonarR.value]
            }
        }

        self.bot_pose = {
                "x": self.pose_data[index][0],
                "y": self.pose_data[index][1],
                "yaw": self.pose_data[index][2]
            }
        
    def simulate(self):
        i = 0
        while i < 1.0 * len(self.time):
            self.update_data(i)
            self.filter.read_data(self.noisy_data)
            est_state, est_covariance = self.filter.estimate()
            #self.filter.predict()
            #est_state, est_covariance = self.filter.mean, self.filter.covariance_mat

            if 0 < i < 5:
                self.filter.mean[enums.state.px.value] = self.bot_pose['x']
                self.filter.mean[enums.state.py.value] = self.bot_pose['y']
                self.filter.mean[enums.state.theta.value] = self.bot_pose['yaw']
                self.filter.mean[enums.state.px_prev.value] = self.pose_data[i-1][0]
                self.filter.mean[enums.state.py_prev.value] = self.pose_data[i-1][1]

            est_measurement = self.filter.measurement_model(est_state)
            est_sonar = est_measurement[:enums.measurement.yaw.value]            
            
            self.logger.log_time(self.time[i])
            self.logger.log_imu(self.noisy_data['imu'])
            self.logger.log_pose(self.bot_pose)
            self.logger.log_sonar(self.noisy_data['sonar'])
            
            self.logger.log_est_sonar(est_sonar)
            self.logger.log_ekf_state(est_state)
            self.logger.log_ekf_covariance(est_covariance)

            i += 1
        self.logger.save_to_csv()
    
    def plot(self):
        self.plotter = plotter.simulationPlotter(self.logger.getLogDirectory())
        self.plotter.plot_error_sonar(show=True)
        self.plotter.plot_estimated_state_with_variance(show=True)

sim = offline_simulator('src\\logs\\2025-06-23_20-14-23')
# sim = offline_simulator('src\\logs\\2025-06-23_20-05-11')
# sim = offline_simulator('src\\logs\\2025-06-23_20-10-31')
sim.simulate()
sim.plot()

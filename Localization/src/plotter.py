import os
import pandas as pd
import matplotlib.pyplot as plt
import enums
import numpy as np

class simulationPlotter:
    def __init__(self, logs_dir):
        # Logs and plots directory setup
        self.logs_dir = logs_dir
        self.plots_dir = "src/plots/" + os.path.basename(os.path.normpath(logs_dir)) + "/"
        os.makedirs(self.plots_dir, exist_ok=True)

        # Load CSV files
        time_data = pd.read_csv(os.path.join(self.logs_dir, 'time.csv'))
        sonar_data = pd.read_csv(os.path.join(self.logs_dir, 'sonar_measurements.csv'))
        est_sonar_data = pd.read_csv(os.path.join(self.logs_dir, 'est_sonar.csv'))
        imu_data = pd.read_csv(os.path.join(self.logs_dir, 'imu_measurements.csv'))
        pose_data = pd.read_csv(os.path.join(self.logs_dir, 'pose.csv'))
        ekf_state_data = pd.read_csv(os.path.join(self.logs_dir, 'ekf_state.csv'))
        cov_diag_data = pd.read_csv(os.path.join(self.logs_dir, 'ekf_cov_diag.csv'))

        # Assign to instance variables
        self.time = time_data['time']
        self.sonar_data = sonar_data[['F', 'FL', 'FR', 'L', 'R']].values
        self.est_sonar_data = est_sonar_data[['F', 'FL', 'FR', 'L', 'R']].values
        self.imu_data = imu_data
        self.pose_data = pose_data[['x', 'y', 'yaw']].values

        self.ekf_state_data = ekf_state_data
        self.cov_diag_data = cov_diag_data

    def plot_sonar_measurements(self, show=False):
        plt.figure(figsize=(10, 6))
        directions = ['F', 'FL', 'FR', 'L', 'R']
        for i, direction in enumerate(directions):
            plt.plot(self.time, self.sonar_data[:, i], label=f'Sonar {direction}')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Sonar Measurement (meters)')
        plt.title('Sonar Measurements Over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(self.plots_dir, 'sonar_measurements.png'))
        if show:
            plt.show()
    
    def plot_error_sonar(self, show=False):
        plt.figure(figsize=(10, 6))
        directions = ['F', 'FL', 'FR', 'L', 'R']
        for i, direction in enumerate(directions):
            plt.plot(self.time, self.sonar_data[:, i] - self.est_sonar_data[:, i], label=f'Sonar {direction}')
        plt.xlabel('Time (seconds)')
        plt.ylabel('error sonar Measurement (meters)')
        plt.title('error sonar Measurements Over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(self.plots_dir, 'est_sonar_.png'))
        if show:
            plt.show()

    def plot_imu_yaw(self, show=False):
        plt.figure(figsize=(10, 4))
        plt.plot(self.time, self.imu_data['yaw'], label='Yaw')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Yaw (radians)')
        plt.title('IMU Yaw Over Time')
        plt.grid(True)
        plt.legend()
        plt.savefig(os.path.join(self.plots_dir, 'imu_yaw.png'))
        if show:
            plt.show()

    def plot_imu_angular_velocity(self, show=False):
        plt.figure(figsize=(10, 4))
        plt.plot(self.time, self.imu_data['angular_velocity_z'], label='Angular Velocity Z')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.title('IMU Angular Velocity Over Time')
        plt.grid(True)
        plt.legend()
        plt.savefig(os.path.join(self.plots_dir, 'imu_angular_velocity.png'))
        if show:
            plt.show()

    def plot_imu_accelerations(self, show=False):
        plt.figure(figsize=(10, 4))
        plt.plot(self.time, self.imu_data['acceleration_x'], label='Acceleration X')
        plt.plot(self.time, self.imu_data['acceleration_y'], label='Acceleration Y')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title('IMU Linear Accelerations Over Time')
        plt.grid(True)
        plt.legend()
        plt.savefig(os.path.join(self.plots_dir, 'imu_accelerations.png'))
        if show:
            plt.show()
        
    def plot_bot_pose(self, show=False):
        plt.figure(figsize=(10, 6))
        indices = ['x', 'y', 'yaw']
        for i, index in enumerate(indices):
            plt.plot(self.time, self.pose_data[:, i], label=f'pose {index}')
        plt.xlabel('Time (seconds)')
        plt.ylabel('bots pose')
        plt.title('Bots pose Over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(self.plots_dir, 'bot_pose.png'))
        if show:
            plt.show()
        
    def plot_estimated_state_with_variance(self, show=False):
        plt.figure(figsize=(12, 8))

        names = ['px', 'py', 'theta']
        true_pose_indices = {'px': 0, 'py': 1, 'theta': 2}  # Corresponds to x, y, yaw
        colors = ['tab:blue', 'tab:green', 'tab:red']

        for i, name in enumerate(names):
            plt.subplot(3, 1, i+1)
            state_index = enums.state[name].value
            variance = self.cov_diag_data.iloc[:, state_index].values
            mean = self.ekf_state_data[name].values

            plt.plot(self.time, mean, label=f'Estimated {name}', color=colors[i])
            plt.fill_between(self.time, mean - np.sqrt(variance), mean + np.sqrt(variance), 
                            color=colors[i], alpha=0.3, label='±1σ')

            true_data = self.pose_data[:, true_pose_indices[name]]
            plt.plot(self.time, true_data, '--', label=f'True {name}', color='black')

            plt.xlabel('Time (s)')
            plt.ylabel(name)
            plt.title(f'{name} Estimate vs Ground Truth with Variance')
            plt.legend()
            plt.grid(True)

        plt.tight_layout()
        plt.savefig(os.path.join(self.plots_dir, 'ekf_estimated_state_with_variance.png'))
        if show:
            plt.show()


import os
import csv
from datetime import datetime
import numpy as np
import enums

class SimulationLogger:
    def __init__(self, base_dir="logs"):
        # Create a folder with the current date and time
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_dir = os.path.join(base_dir, timestamp)
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Data buffers
        self.time_log = []
        self.sonar_measurements_log = []
        self.imu_measurements_log = []
        self.pose_log = []
        self.state_log = []
        self.cov_diag_log = []

        self.est_sonar_log = []
    
    def getLogDirectory(self):
        return self.log_dir

    def log_time(self, time):
        self.time_log.append([time])

    def log_sonar(self, sonar_data):
        """
        sonar_data: dict with keys ['F', 'FL', 'FR', 'L', 'R']
        """
        row = [sonar_data.get(key, 0.0) for key in ['F', 'FL', 'FR', 'L', 'R']]
        self.sonar_measurements_log.append(row)
    
    def log_est_sonar(self, est_sonar_data):
        self.est_sonar_log.append(est_sonar_data)

    def log_imu(self, imu_data):
        """
        imu_data: dict with keys ['yaw', 'angular_velocity_z', 'acceleration_x', 'acceleration_y']
        """
        row = [imu_data.get(key, 0.0) for key in ['yaw', 'angular_velocity_z', 'acceleration_x', 'acceleration_y']]
        self.imu_measurements_log.append(row)

    def log_pose(self, pose_data):
        """
        pose_data: dict with keys ['x', 'y', 'yaw']
        """
        row = [pose_data.get(key, 0.0) for key in ['x', 'y', 'yaw']]
        self.pose_log.append(row)
    
    def log_ekf_state(self, state):
        """
        state: 1D numpy array or list representing the EKF state vector
        """
        self.state_log.append(state.tolist())

    def log_ekf_covariance(self, P):
        """
        P: 2D numpy array representing the EKF covariance matrix
        Logs only the diagonal (variances)
        """
        diag = np.diag(P).tolist()
        self.cov_diag_log.append(diag)

    def save_to_csv(self):
        # Save time log
        with open(os.path.join(self.log_dir, "time.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time'])
            writer.writerows(self.time_log)

        # Save sonar log
        with open(os.path.join(self.log_dir, "sonar_measurements.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['F', 'FL', 'FR', 'L', 'R'])
            writer.writerows(self.sonar_measurements_log)

        # Save IMU log
        with open(os.path.join(self.log_dir, "imu_measurements.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['yaw', 'angular_velocity_z', 'acceleration_x', 'acceleration_y'])
            writer.writerows(self.imu_measurements_log)

        with open(os.path.join(self.log_dir, "pose.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw'])
            writer.writerows(self.pose_log)

        # Save EKF state log with headers from the Enum
        with open(os.path.join(self.log_dir, "ekf_state.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            # Write headers using the enum names, sorted by value
            headers = [s.name for s in sorted(enums.state, key=lambda x: x.value)]
            writer.writerow(headers)
            writer.writerows(self.state_log)

        # Save EKF covariance diagonal
        with open(os.path.join(self.log_dir, "ekf_cov_diag.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            # Write headers using the enum names, sorted by value
            headers = [s.name for s in sorted(enums.state, key=lambda x: x.value)]
            writer.writerow(headers)
            writer.writerows(self.cov_diag_log)
        
        with open(os.path.join(self.log_dir, "est_sonar.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['F', 'FL', 'FR', 'L', 'R'])
            writer.writerows(self.est_sonar_log)

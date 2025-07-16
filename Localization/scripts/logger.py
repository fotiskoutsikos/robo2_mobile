import os
import csv
from datetime import datetime

class SimulationLogger:
    def __init__(self, base_dir="logs"):
        # Create a folder with the current date and time
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_dir = os.path.join(base_dir, timestamp)
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Data buffers
        self.time_log = []
        self.sonar_measurements_log = []
        self.desired_direction_log = []
        self.error_angle_log = []
        self.linear_velocity_log = []
        self.angular_velocity_log = []
        self.error_fix_vector_log = []
        self.tangent_vector_log = []
        self.error_log = []
        self.parameters_log = []

    def log_time(self, time):
        self.time_log.append([time])  # Single-column list
    
    def log_sonar_measurements(self, sonar_measurements):
        # Extract the sonar measurements and log them
        self.sonar_measurements_log.append([
            sonar_measurements.get('F', 0),
            sonar_measurements.get('FL', 0),
            sonar_measurements.get('FR', 0),
            sonar_measurements.get('L', 0),
            sonar_measurements.get('R', 0)
        ])

    def log_desired_direction(self, desired_direction):
        self.desired_direction_log.append(desired_direction)

    def log_error_angle(self, error_angle):
        self.error_angle_log.append([error_angle])

    def log_linear_velocity(self, linear_velocity):
        self.linear_velocity_log.append([linear_velocity])

    def log_angular_velocity(self, angular_velocity):
        self.angular_velocity_log.append([angular_velocity])
    
    def log_error_fix_vector(self, error_fix_vector):
        self.error_fix_vector_log.append(error_fix_vector)
    
    def log_tangent_vector(self, tangent_vector):
        self.tangent_vector_log.append(tangent_vector)
    
    def log_error(self, error):
        self.error_log.append([error])
    
    def log_parameters(self, v_min, v_max, weight, gain, slow_factor):
        self.parameters_log.append([v_min, v_max, weight, gain, slow_factor])

    def save_to_csv(self):
        # Save time log
        with open(os.path.join(self.log_dir, "time.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time'])
            writer.writerows(self.time_log)

        # Save sonar measurements log
        with open(os.path.join(self.log_dir, "sonar_measurements.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['F', 'FL', 'FR', 'L', 'R'])
            writer.writerows(self.sonar_measurements_log)
        
        # Save desired direction log
        with open(os.path.join(self.log_dir, "desired_direction.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['desired_direction_x', 'desired_direction_y'])
            writer.writerows(self.desired_direction_log)
        
        # Save error angle log
        with open(os.path.join(self.log_dir, "error_angle.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['error_angle'])
            writer.writerows(self.error_angle_log)

        # Save linear velocity log
        with open(os.path.join(self.log_dir, "linear_velocity.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['linear_velocity'])
            writer.writerows(self.linear_velocity_log)

        # Save angular velocity log
        with open(os.path.join(self.log_dir, "angular_velocity.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['angular_velocity'])
            writer.writerows(self.angular_velocity_log)
        
        with open(os.path.join(self.log_dir, "error_fix_vector.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['error_fix_vector_x', 'error_fix_vector_y'])
            writer.writerows(self.error_fix_vector_log)
        
        with open(os.path.join(self.log_dir, "tangent_vector.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['tangent_vector_x', 'tangent_vector_y'])
            writer.writerows(self.tangent_vector_log)
        
        with open(os.path.join(self.log_dir, "error.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['error'])
            writer.writerows(self.error_log)
        
        with open(os.path.join(self.log_dir, "parameters.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['v_min', 'v_max', 'weight', 'gain', 'slow_factor'])
            writer.writerows(self.parameters_log)


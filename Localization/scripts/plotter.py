import os
import pandas as pd
import matplotlib.pyplot as plt

class simulationPlotter:
    def __init__(self, logs_dir):
        # Logs and plots directory setup
        self.logs_dir = logs_dir
        self.plots_dir = "plots/" + os.path.basename(os.path.normpath(logs_dir)) + "/"
        os.makedirs(self.plots_dir, exist_ok=True)

        # Extract values from logger CSV files
        time_data = pd.read_csv(os.path.join(self.logs_dir, 'time.csv'))
        sonar_data = pd.read_csv(os.path.join(self.logs_dir, 'sonar_measurements.csv'))
        desired_direction_data = pd.read_csv(os.path.join(self.logs_dir, 'desired_direction.csv'))
        error_angle_data = pd.read_csv(os.path.join(self.logs_dir, 'error_angle.csv'))
        linear_velocity_data = pd.read_csv(os.path.join(self.logs_dir, 'linear_velocity.csv'))
        angular_velocity_data = pd.read_csv(os.path.join(self.logs_dir, 'angular_velocity.csv'))
        error_fix_vector_data = pd.read_csv(os.path.join(self.logs_dir, 'error_fix_vector.csv'))
        tangent_vector_data = pd.read_csv(os.path.join(self.logs_dir, 'tangent_vector.csv'))
        error_data = pd.read_csv(os.path.join(self.logs_dir, 'error.csv'))

        # Assign data to instance variables
        self.time = time_data['time']
        self.sonar_data = sonar_data[['F', 'FL', 'FR', 'L', 'R']].values
        self.desired_direction = desired_direction_data[['desired_direction_x', 'desired_direction_y']].values
        self.error_angle = error_angle_data['error_angle'].values
        self.linear_velocity = linear_velocity_data['linear_velocity'].values
        self.angular_velocity = angular_velocity_data['angular_velocity'].values
        self.error_fix_vector = error_fix_vector_data[['error_fix_vector_x', 'error_fix_vector_y']].values
        self.tangent_vector = tangent_vector_data[['tangent_vector_x', 'tangent_vector_y']].values
        self.error = error_data['error'].values
        

    def plot_sonar_measurements(self, show=False):
        # Plot sonar measurements over time
        plt.figure(figsize=(10, 6))
        directions = ['F', 'FL', 'FR', 'L', 'R']
        for i, direction in enumerate(directions):
            plt.plot(self.time, self.sonar_data[:, i], label=f'Sonar {direction}')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Sonar Measurement (meters)')
        plt.title('Sonar Measurements Over Time')
        plt.legend()
        plt.grid(True)
        
        # Save and optionally show the plot
        plt.savefig(os.path.join(self.plots_dir, 'sonar_measurements.png'))
        if show:
            plt.show()

    def plot_desired_direction(self, show=False):
        # Plot desired direction (x, y components) over time
        plt.figure(figsize=(10, 6))
        plt.plot(self.time, self.desired_direction[:, 0], label='Desired X Direction')
        plt.plot(self.time, self.desired_direction[:, 1], label='Desired Y Direction')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Direction (unit vectors)')
        plt.title('Desired Direction Over Time')
        plt.legend()
        plt.grid(True)
        
        # Save and optionally show the plot
        plt.savefig(os.path.join(self.plots_dir, 'desired_direction.png'))
        if show:
            plt.show()

    def plot_error_angle(self, show=False):
        # Plot error angle over time
        plt.figure(figsize=(10, 6))
        plt.plot(self.time, self.error_angle, label='Error Angle', color='r')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Error Angle (radians)')
        plt.title('Error Angle Over Time')
        plt.grid(True)
        
        # Save and optionally show the plot
        plt.savefig(os.path.join(self.plots_dir, 'error_angle.png'))
        if show:
            plt.show()

    def plot_velocities(self, show=False):
        # Plot linear and angular velocities over time
        plt.figure(figsize=(10, 6))
        plt.plot(self.time, self.linear_velocity, label='Linear Velocity', color='g')
        plt.plot(self.time, self.angular_velocity, label='Angular Velocity', color='b')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Velocity (m/s or rad/s)')
        plt.title('Linear and Angular Velocities Over Time')
        plt.legend()
        plt.grid(True)
        
        # Save and optionally show the plot
        plt.savefig(os.path.join(self.plots_dir, 'velocities.png'))
        if show:
            plt.show()
    
    def plot_error_fix_vector(self, show=False):
        # Plot fix-vector (x, y components) over time
        plt.figure(figsize=(10, 6))
        plt.plot(self.time, self.error_fix_vector[:, 0], label='fix_vector X ')
        plt.plot(self.time, self.error_fix_vector[:, 1], label='fix_vector Y ')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Direction (unit vectors)')
        plt.title('fix-vector Over Time')
        plt.legend()
        plt.grid(True)
        
        # Save and optionally show the plot
        plt.savefig(os.path.join(self.plots_dir, 'error_fix_vector.png'))
        if show:
            plt.show()

    def plot_tangent_vector(self, show=False):
        # Plot tangent vactor (x, y components) over time
        plt.figure(figsize=(10, 6))
        plt.plot(self.time, self.tangent_vector[:, 0], label='tangent X')
        plt.plot(self.time, self.tangent_vector[:, 1], label='tangent Y')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('tangent (unit vectors)')
        plt.title('tangent vector Over Time')
        plt.legend()
        plt.grid(True)
        
        # Save and optionally show the plot
        plt.savefig(os.path.join(self.plots_dir, 'tangent_vector.png'))
        if show:
            plt.show()

    def plot_error(self, show=False):
        plt.figure(figsize=(10, 6))
        plt.plot(self.time, self.error, label='error')
        plt.xlabel('Time (seconds)')
        plt.ylabel('error')
        plt.title('distance error Over Time')
        plt.grid(True)
        plt.savefig(self.plots_dir + 'error.png')
        if show:
            plt.show()

logs_path = 'logs/2025-05-11_11-52-38/'
plotter = simulationPlotter(logs_path)

plotter.plot_sonar_measurements(show=False)
plotter.plot_desired_direction(show=False)
plotter.plot_error_angle(show=False)
plotter.plot_velocities(show=False)
plotter.plot_error_fix_vector(show=False)
plotter.plot_tangent_vector(show=False)
plotter.plot_error(show=True)

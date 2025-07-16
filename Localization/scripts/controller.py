
import numpy as np
import utils
import logger

class mobibot_controller:
    def __init__(self):
        # AM1 + AM2 = 3 + 2 = 5 (odd) -> CCW
        # 5 mod pi = 5 - pi = 1.8584 = yaw_init
        self.motion = "CCW"
        self.distance_from_obstacles = 0.5

        self.sonar_measurements = {
            "F": 0,
            "FL": 0,
            "FR": 0,
            "L": 0,
            "R": 0
        }

        self.sonar_angles = {
            "F": 0,
            "FL": np.pi / 4,
            "FR": - np.pi / 4,
            "L": np.pi / 2,
            "R": - np.pi / 2
        }

        self.v_min = 0.001
        self.v_max = 0.40
        self.error_fix_weight = 2**3
        self.controller_gain = 0.5
        self.slow_factor = 30.0

        self.time = 0.0
        self.logger = logger.SimulationLogger()
        self.logger.log_parameters(self.v_min, self.v_max, self.error_fix_weight, self.controller_gain, self.slow_factor)
        
        return

    def update_sonar_measurements(self, new_sonar_measurements):
        self.sonar_measurements = new_sonar_measurements
        self.logger.log_sonar_measurements(self.sonar_measurements)
        return
    
    def update_time(self, time):
        self.time = time
        self.logger.log_time(self.time)
        return
    
    def get_observed_point(self, direction):
        return utils.polar_to_cartesian(self.sonar_measurements[direction],self.sonar_angles[direction])
    
    def get_observed_points(self):
        p1 = self.get_observed_point("L")
        p2 = self.get_observed_point("FL")
        p3 = self.get_observed_point("F")
        p4 = self.get_observed_point("FR")
        p5 = self.get_observed_point("R")
        return np.array([p1, p2, p3, p4, p5])
    
    def get_error_fix_vector(self, min_vector):
        error = self.distance_from_obstacles - np.linalg.norm(min_vector)
        error_fix_vec = np.array([0, 0])
        if error < 0:
            error_fix_vec = min_vector
        else:
            if self.motion == "CW":
                error_fix_vec = np.array([0, -1.0])
            elif self.motion == 'CCW':
                error_fix_vec = np.array([0, 1.0])
        
        error_fix_vec = utils.normalize(error_fix_vec)
        return error_fix_vec
    
    def get_trajectory_tangent_vector(self, min_vector):
        tangent_vector = utils.normalize(min_vector)
        if self.motion == 'CW':
            tangent_vector = utils.rotate_vector(tangent_vector, - np.pi / 2)
        elif self.motion == 'CCW':
            tangent_vector = utils.rotate_vector(tangent_vector, np.pi / 2)
        
        return tangent_vector
    
    def combine_directions(self, error_fix_vec, motion_maint_vec, distance_error):
        weight = self.error_fix_weight * distance_error ** 2
        desired_direction = weight * error_fix_vec + motion_maint_vec
        return utils.normalize(desired_direction)
    
    def desired_direction(self):
        line_segs = utils.points_to_lines(self.get_observed_points())
        bot_position = np.array([0, 0])

        vectors = []
        for line_segment in line_segs:
            vector = utils.closest_vector_to_segment(bot_position, line_segment[0], line_segment[1])
            vectors.append(vector)
        vectors = np.array(vectors)
        lengths = np.linalg.norm(vectors, axis=1)
        min_index = np.argmin(lengths)
        min_vector = vectors[min_index]

        error = self.distance_from_obstacles - np.linalg.norm(min_vector)
        self.logger.log_error(error)
        
        vec1 = self.get_error_fix_vector(min_vector)
        vec2 = self.get_trajectory_tangent_vector(min_vector)
        
        desired_direction = self.combine_directions(vec1, vec2, error)

        self.logger.log_error_fix_vector(vec1)
        self.logger.log_tangent_vector(vec2)
        self.logger.log_desired_direction(desired_direction)
        
        return desired_direction

    def control_law(self):
        des_direction = self.desired_direction()
        current_direction = np.array([1, 0])
        error_angle = utils.angle_between_vectors(des_direction, current_direction)
        rotational_speed = self.controller_gain * error_angle
        if self.motion == 'CCW':
            rotational_speed = - rotational_speed
        
        heading_speed = self.v_min + (1 / (1+ self.slow_factor * error_angle**2)) * (self.v_max - self.v_min)
        
        self.logger.log_angular_velocity(rotational_speed)
        self.logger.log_linear_velocity(heading_speed)
        self.logger.log_error_angle(error_angle)

        return np.array([heading_speed, rotational_speed])
    
    def log_data(self):
        self.logger.save_to_csv()
        print("saved_log")



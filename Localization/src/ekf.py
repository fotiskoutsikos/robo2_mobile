import numpy as np
import enums
import utils
import map
import sonar

class ekf:
    def __init__(self, rate, mean, covariance_mat):
        self.dt = 1 / rate
        print("dt = ", self.dt)
        self.mean = mean
        self.covariance_mat = covariance_mat
        
        self.measurement = np.zeros(len(enums.measurement))
        self.input = np.zeros(len(enums.input))

        self.sonar = sonar.sonar_model()
        self.border = 2.0
        self.map = map.map_model()

        self.std_acc = 0.02 * self.dt ** 2
        self.std_ang_vel = 0.2 * self.dt
        self.std_theta = 0.002
        self.std_sonar = 0.01

        # uncorellated noise
        self.R = np.diag([self.std_acc**2]*2 + [self.std_ang_vel**2] + [0.001]*2)
        self.Q = np.diag([self.std_sonar**2]*5 + [self.std_theta**2])
        
    def state_transition_model(self, prev_state, input):
        next_state = np.zeros(len(enums.state))
        measured_acc = np.array([input[enums.input.acc_x.value], input[enums.input.acc_y.value]])
        accel = utils.rotate_vector(measured_acc, prev_state[enums.state.theta.value])
        
        vx = (prev_state[enums.state.px.value] - prev_state[enums.state.px_prev.value]) / self.dt
        vx += accel[0] * self.dt
        vy = (prev_state[enums.state.py.value] - prev_state[enums.state.py_prev.value]) / self.dt
        vy += accel[1] * self.dt

        max_vel = 10.0
        vx = np.clip(vx, -max_vel, max_vel)
        vy = np.clip(vy, -max_vel, max_vel)

        vel = np.array([vx, vy])
        direction = utils.rotate_vector(np.array([1.0, 0.0]), prev_state[enums.state.theta.value])
        vel_projection = np.dot(vel, direction) * direction
        vx = vel_projection[0]
        vy = vel_projection[1]
        
        next_state[enums.state.px.value] = prev_state[enums.state.px.value] + vx * self.dt
        next_state[enums.state.py.value] = prev_state[enums.state.py.value] + vy * self.dt

        next_state[enums.state.px_prev.value] = prev_state[enums.state.px.value]
        next_state[enums.state.py_prev.value] = prev_state[enums.state.py.value]
        dtheta = input[enums.input.ang_vel.value] * self.dt
        
        next_state[enums.state.theta.value] = utils.wrap_angle(prev_state[enums.state.theta.value] + dtheta)
        
        return next_state
    
    def sonar_distance(self, pos, direction):
        dist = self.map.get_distance(pos, direction)
        if dist is None:
            # print("bad distance")
            # print("mean = ", self.mean)
            
            return 0.01
        else:
            return min(dist, self.sonar.max_distance)
    
    def measurement_model(self, state):
        measurements = np.zeros(len(enums.measurement))

        measurements[enums.measurement.yaw.value] = state[enums.state.theta.value]

        position = np.array([state[enums.state.px.value], state[enums.state.py.value]])
        directions = np.zeros((enums.measurement.yaw.value, 2))
        positions = np.zeros((enums.measurement.yaw.value, 2))
        theta = state[enums.state.theta.value]
        for i in range(len(directions)):
            directions[i] = utils.rotate_vector(np.array([1.0, 0.0]), theta + self.sonar.angles[i])
            positions[i] = position + utils.rotate_vector(self.sonar.offset[i], theta)
            measurements[i] = self.sonar_distance(positions[i], directions[i])

        return measurements
    
    def state_transition_jacobian(self, state, input, epsilon=1e-5):
        n = len(state)
        F = np.zeros((n, n))
        fx = self.state_transition_model(state, input)

        for st in enums.state:
            state_perturbed = np.copy(state)
            state_perturbed[st.value] += epsilon
            fx_perturbed = self.state_transition_model(state_perturbed, input)
            F[:, st.value] = (fx_perturbed - fx) / epsilon

        return F
    
    def measurement_model_jacobian(self, state, epsilon=1e-5):
        n = len(state)
        m = len(enums.measurement)
        H = np.zeros((m, n))
        hx = self.measurement_model(state)

        for st in enums.state:
            state_perturbed = np.copy(state)
            state_perturbed[st.value] += epsilon
            hx_perturbed = self.measurement_model(state_perturbed)
            H[:, st.value] = (hx_perturbed - hx) / epsilon

        return H
    
    def read_data(self, data):
        acc_x = data['imu']['acceleration_x']
        acc_y = data['imu']['acceleration_y']
        ang_vel = data['imu']['angular_velocity_z']
        yaw = data['imu']['yaw']

        sonarF = data['sonar']['F']
        sonarFL = data['sonar']['FL']
        sonarFR = data['sonar']['FR']
        sonarL = data['sonar']['L']
        sonarR = data['sonar']['R']

        self.input[enums.input.acc_x.value] = acc_x
        self.input[enums.input.acc_y.value] = acc_y
        self.input[enums.input.ang_vel.value] = ang_vel

        self.measurement[enums.measurement.yaw.value] = yaw
        self.measurement[enums.measurement.sonarF.value] = sonarF
        self.measurement[enums.measurement.sonarFL.value] = sonarFL
        self.measurement[enums.measurement.sonarFR.value] = sonarFR
        self.measurement[enums.measurement.sonarL.value] = sonarL
        self.measurement[enums.measurement.sonarR.value] = sonarR
        
    def predict(self):
        F = self.state_transition_jacobian(self.mean, self.input)
        self.mean = self.state_transition_model(self.mean, self.input)
        self.covariance_mat = F @ self.covariance_mat @ F.T + self.R

    def update(self):
        H = self.measurement_model_jacobian(self.mean)
        z_hat = self.measurement_model(self.mean)
        y = self.measurement - z_hat


        yaw_idx = enums.measurement.yaw.value
        y[yaw_idx] = utils.wrap_angle(y[yaw_idx])

        S = H @ self.covariance_mat @ H.T + self.Q
        K = self.covariance_mat @ H.T @ np.linalg.inv(S)

        self.mean = self.mean + K @ y

        self.mean[enums.state.theta.value] = utils.wrap_angle(self.mean[enums.state.theta.value])
        self.covariance_mat = (np.eye(len(self.mean)) - K @ H) @ self.covariance_mat

    def estimate(self):
        self.predict()
        self.update()
        return self.mean, self.covariance_mat


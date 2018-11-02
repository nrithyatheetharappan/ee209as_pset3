
from laser_range_finder import DistanceGenerator
import numpy as np
import math

class World:
    def __init__(self, Length, Width, radius, L):
        self.Length = Length
        self.Width = Width
        self.radius = radius
        self.L = L

        
class car_simulation(DistanceGenerator):
    def __init__(self, r, phi_1, phi_2, L, dt):
        self.theta_t_measured = 0
        self.x_t_measured = 0
        self.y_t_measured = 0
        self.omega_t_measured = 0
        self.v_t_measured = 0
        self.r = r
        self.phi_1 = phi_1
        self.phi_2 = phi_2
        self.L = L
        self.sensor_output = np.zeros(4)
        self.bias = 0
        self.dt = dt

    def get_simulation(self,i):
        w_omega_t = np.random.normal(0, 0.066)
        w_v_t = np.random.normal(0, 0.066)
        self.omega_t_measured = ((self.r*self.phi_1) - (self.r*self.phi_2))/self.L + w_omega_t
        self.v_t_measured = ((self.r*self.phi_1) + (self.r*self.phi_2))/2 + w_v_t
        self.x_t__measured = self.x_t_measured + self.v_t_measured * math.cos(self.theta_t_measured) * self.dt
        self.y_t_measured = self.y_t_measured + self.v_t_measured * math.sin(self.theta_t_measured) * self.dt
        self.theta_t_measured = self.theta_t_measured + self.omega_t_measured * self.dt
        self.bias = self.bias
        z = np.zeros((1,6))
        z[i][0] = self.x_t_measured
        z[i][1] = self.y_t_measured
        z[i][2] = self.v_t_measured
        z[i][3] = self.theta_t_measured
        z[i][4] = self.omega_t_measured
        z[i][5] = self.bias

    def get_sensor_simulation(self):
        d1 = DistanceGenerator(self.x_t_measured, self.y_t_measured, self.theta_t_measured)
        d2 = DistanceGenerator(self.x_t_measured, self.y_t_measured, self.theta_t_measured + np.pi)
        distance_one = d1.laser_output() + np.random.normal(0, .04)
        distance_two = d2.laser_output() + np.random.normal(0, .04)
        theta_t_measured = self.theta_t_measured + np.random.normal(0, .001)
        omega_t_measured = self.omega_t_measured + np.random.normal(0, .001)
        self.sensor_output[0] = distance_one
        self.sensor_output[1] = distance_two
        self.sensor_output[2] = theta_t_measured
        self.sensor_output[3] = omega_t_measured
        return self.sensor_output

def find_F_t(F_t,theta_t_hat, v_t_hat, dt):
    F_t[0][3] = -1 * v_t_hat * np.sin(theta_t_hat) * dt
    F_t[1][3] =  v_t_hat * math.cos(theta_t_hat) * dt
    F_t[0][0],F_t[1][1],F_t[3][3],F_t[5][5] = 1,1,1,1
    return F_t


def find_W_t(W_t,theta_t_hat, dt):
    W_t[0][0] = math.cos(theta_t_hat) * dt
    W_t[1][0] = math.sin(theta_t_hat) * dt
    W_t[3][1]= dt
    W_t[2][0],W_t[4][1] = 1,1
    return W_t


def find_H_t(H_t,observation,z_bar,landmark_values):
    x_bar = z_bar[0]
    y_bar = z_bar[2]
    x_l1_bar = landmark_values[0][0]
    y_l1_bar = landmark_values[0][1]
    x_l2_bar = landmark_values[1][0]
    y_l2_bar = landmark_values[1][1]
    d1_bar = observation[0]
    d2_bar = observation[1]
    H_t[0][0] = (x_bar - x_l1_bar)/d1_bar
    H_t[0][1] = (y_bar - y_l1_bar)/d1_bar
    H_t[1][1]= (y_bar - y_l2_bar)/d2_bar
    H_t[1][0]= (x_bar - x_l2_bar)/d2_bar
    H_t[2][3],H_t[3][4] = 1,1
    return H_t


class EKF(car_simulation):
    def __init__(self, phi_1, phi_2, dt, L, Q, R, r):
        car_simulation.__init__(self, r, phi_1, phi_2, L, dt)
        self.z_hat = np.zeros((1,6))
        self.z_hat[0][2] = ((r * phi_1) + (r * phi_2)) / 2
        self.z_hat[0][4] = ((r*phi_1) - (r*phi_2))/L
        self.z_bar = np.zeros(6)
        self.F_t = np.zeros((6,6))
        self.W_t = np.zeros((6,2))
        self.sigma_hat = np.zeros((6,6))
        self.sigma_bar = np.zeros((6,6))
        self.H_t = np.zeros((4,6))
        self.observation_model = np.zeros(4)
        self.Q = Q
        self.R = R
        self.error = 0
        
    def time_propagation_update(self):
        x_t_hat = self.z_hat[0]
        y_t_hat = self.z_hat[1]
        v_t_hat = self.z_hat[2]
        theta_t_hat = self.z_hat[3]
        omega_t_hat = self.z_hat[4]
        bias        = self.z_hat[5]

        x_t_plus_one_bar = x_t_hat + v_t_hat* math.cos(omega_t_hat) * self.dt
        y_t_plus_one_bar = y_t_hat + v_t_hat * math.cos(omega_t_hat) * self.dt
        v_t_plus_one_bar = v_t_hat
        theta_t_plus_one_bar = theta_t_hat + omega_t_hat * self.dt
        omega_t_plus_one_bar = omega_t_hat
        bias_plus_one_bar = bias

        self.z_bar[0] = x_t_plus_one_bar
        self.z_bar[1] = y_t_plus_one_bar
        self.z_bar[2] = v_t_plus_one_bar
        self.z_bar[3] = theta_t_plus_one_bar
        self.z_bar[4] = omega_t_plus_one_bar
        self.z_bar[5] = bias_plus_one_bar

    def time_linearization(self):
        v_t_hat = self.z_hat[2]
        theta_t_hat = self.z_hat[3]
        self.F_t = find_F_t(self.F_t,theta_t_hat, v_t_hat, self.dt)
        self.W_t = find_W_t(self.W_t, theta_t_hat, self.dt)

    def covariance_update(self):
        sigma_t_plus_one_temp1 = np.dot(self.F_t,self.sigma_hat)
        sigma_t_plus_one_temp2 = np.dot(sigma_t_plus_one_temp1,self.F_t.transpose())
        sigma_t_plus_one_temp3 = np.dot(self.W_t,self.Q)
        sigma_t_plus_one_temp4 = np.dot(sigma_t_plus_one_temp3,self.W_t.transpose())
        self.sigma_bar = sigma_t_plus_one_temp2 + sigma_t_plus_one_temp4

    def get_observation_model(self):
        x_bar = self.z_bar[0]
        y_bar = self.z_bar[1]
        theta_bar = self.z_bar[3]
        omega_bar = self.z_bar[4]
        d1_bar = DistanceGenerator(x_bar, y_bar, theta_bar)
        d2_bar = DistanceGenerator(x_bar, y_bar, theta_bar + np.pi)
        distance_one_bar = d1_bar.laser_output()
        distance_two_bar = d2_bar.laser_output()
        self.observation_model[0] = distance_one_bar + np.random.normal(0, .04)
        self.observation_model[1] = distance_two_bar + np.random.normal(0, .04)
        self.observation_model[2] = theta_bar + np.random.normal(0, .001)
        self.observation_model[3] = omega_bar + np.random.normal(0, .001)

    def observation_linearization(self):
        x_bar = self.z_bar[0]
        y_bar = self.z_bar[1]
        theta_bar = self.z_bar[3]
        d1_bar = DistanceGenerator(x_bar, y_bar, theta_bar)
        d2_bar = DistanceGenerator(x_bar, y_bar, theta_bar + np.pi)
        landmark_values = [d1_bar.get_landmarks(), d2_bar.get_landmarks()]
        self.H_t = find_H_t(self.H_t,self.observation_model,self.z_bar,landmark_values)


    def kalman_gain_value(self):
        inner_temp1 = np.dot(self.H_t,self.sigma_bar)
        inner_temp2 = np.dot(inner_temp1,self.H_t.transpose())
        inner_temp3 = inner_temp2 + self.R
        inner_temp4 = np.linalg.inv(inner_temp3)
        outer_temp = np.dot(self.sigma_bar,self.H_t.transpose())
        self.kalman_gain = np.dot(outer_temp, inner_temp4)

    def error_calculation(self):
        z = self.get_sensor_simulation()
        self.error = z - self.observation_model
        
    def conditional_mean(self, i):
        temp_product = np.dot(self.kalman_gain,self.error)
        self.z_hat[i] = self.z_bar + temp_product

    def observation_update_covariance(self):
        inner_product1 = np.dot(self.kalman_gain,self.H_t)
        inner_product2 = np.dot(inner_product1,self.sigma_bar)
        self.sigma_hat = self.sigma_bar - inner_product2


class DistanceGenerator:
    def __init__(self, x, y, theta):
        self.location = np.array([x, y])
        self.x_line_vertical = 500.0
        self.y_line_horizontal = 750.0
        self.phi = theta + np.pi/2
        self.nhat = np.array([np.cos(self.phi), np.sin(self.phi)])
        self.slope = np.tan(self.phi)
        self.y_int = y - self.slope*x
        self.direction_check = np.zeros(4)
        self.distance = np.zeros(2)
        self.min_distance = 0
        self.test_points = [[0, 0], [0, 0]]
        self.landmark_point = np.zeros(2)

    def direction_calc(self, X):
        rho = np.dot(X, self.nhat)/np.linalg.norm(X)
        value = np.round(rho, 0) + 1
        return value

    def distance_calc(self, point):
        distance = np.sqrt((point[0] - self.location[0])**2 + (point[1] - self.location[1])**2)
        return distance

    def valid_points(self):
        X = np.zeros((4, 2))
        # start with the y-int
        X[0] = np.array([0.0, self.y_int])
        # x-int
        X[1] = np.array([-self.y_int/self.slope, 0.0])
        # vertical line
        X[2] = np.array([self.x_line_vertical, self.slope*self.x_line_vertical + self.y_int])
        # horizontal line
        X[3] = np.array([(self.y_line_horizontal-self.y_int)/self.slope, self.y_line_horizontal])
        X_relative = X - self.location
        i = 0
        while np.linalg.norm(self.direction_check) <= 2:
            self.direction_check[i] = self.direction_calc(X_relative[i])
            i += 1
        direction_index = [k for k, e in enumerate(self.direction_check) if e != 0]
        self.test_points = [X[direction_index[0]], X[direction_index[1]]]

    def laser_output(self):
        for m, n in enumerate(self.test_points):
            self.distance[m] = self.distance_calc(n)
        self.min_distance = np.min(self.distance)

    def get_landmarks(self):
        landmark = np.argmin(self.distance)
        self.landmark_point = self.test_points[np.asscalar(landmark)]




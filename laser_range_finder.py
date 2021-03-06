
import numpy as np

class DistanceGenerator(object):
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
        return self.test_points

    def laser_output(self):
        test_points = self.valid_points()
        for m, n in enumerate(test_points):
            self.distance[m] = self.distance_calc(n)
        self.min_distance = np.min(self.distance)
        return self.min_distance


class SensorSimulation(DistanceGenerator):
    def __init__(self, x, y, theta, omega):
        #DistanceGenerator.__init__(self, x, y, theta)
        super(SensorSimulation, self).__init__(x, y, theta)
        self.distance_one = self.laser_output() + np.random.normal(0, .04)
        super(SensorSimulation, self).__init__(x, y, theta+np.pi/2)
        self.distance_two = self.laser_output() + np.random.normal(0, .04)
        self.theta = theta + np.random.normal(0, .001)
        self.omega = omega + np.random.normal(0, .001)
        self.sensor_simulation = np.array([self.distance_one, self.distance_two, self.theta, self.omega])


class ObservationModel(DistanceGenerator):
    def __init__(self, x_bar, y_bar, theta_bar, omega_bar):
        D1_bar = DistanceGenerator.__init__(self, x_bar, y_bar, theta_bar)
        D2_bar = DistanceGenerator.__init__(self, x_bar, y_bar, theta_bar + np.pi)
        self.distance_one_bar = D1_bar.laser_output() + np.random.normal(0, .04)
        self.distance_two_bar = D2_bar.laser_output() + np.random.normal(0, .04)
        self.theta_bar = theta_bar + np.random.normal(0, .001)
        self.omega_bar = omega_bar + np.random.normal(0, .001)
        self.observation_model = np.array([self.distance_one_bar, self.distance_two_bar, self.theta_bar, self.omega_bar])


if __name__ == '__main__':
    # plot the trajectories
    distance = DistanceGenerator(300, 400, -np.pi/6 + np.pi + np.pi/2)
    #points = distance.valid_points()
    # print points
    d = distance.laser_output()
    print d
    this = SensorSimulation(300, 400, -np.pi/6 + np.pi + np.pi/2, 1)
    print this.sensor_simulation























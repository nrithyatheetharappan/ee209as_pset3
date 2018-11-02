
import numpy as np


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
        self.min_distance = 0.0

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
        test_points = [X[direction_index[0]], X[direction_index[1]]]
        return test_points

    def laser_output(self, point):
        for m, n in enumerate(point):
            self.distance[m] = self.distance_calc(n)
            self.min_distance = np.min(self.distance)
        return self.min_distance





























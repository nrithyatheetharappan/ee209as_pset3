"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt
from pset3_test import World
from pset3_test import car_simulation
from pset3_test import EKF


def plot_covariance_ellipse(z_hat, sigma_hat):
    w,v = np.linalg.eig(sigma_hat)
    print('all')
    print(w)
    Pxy = sigma_hat[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    print('some')
    print(eigval)
    a = math.sqrt(np.absolute(eigval[bigind]))
    b = math.sqrt(np.absolute(eigval[smallind]))
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + z_hat[0][0]).flatten()
    py = np.array(fx[1, :] + z_hat[0][1]).flatten()
    plt.plot(px, py, "--r")
    plt.ylim(250, 700)
    plt.xlim(280, 320)

def main():
    k = 0
    input1 = 8
    input2 = 8
    sim_time = 4 # the car travels at 20 mm per second
    wheel_radius = 20
    width = 500
    height = 750
    wheel_base = 85
    time_step = .01
    x_i = 300
    y_i = 0
    theta_i = 0
    track = World(height, width, wheel_radius, wheel_base)
    car = car_simulation(wheel_radius, input1, input2, wheel_base, time_step, sim_time, x_i, y_i, theta_i)
    car_state = car.get_simulation()
    car_sensor_readout = car.get_sensor_simulation()
    estimator = EKF(input1, input2, time_step, wheel_base, wheel_radius, sim_time, x_i, y_i, theta_i)
    z_hat_list = np.zeros((1, 6))

    # State Vector [x y yaw v]'
    xEst = np.array([[x_i, y_i, 0,  theta_i, 0, 0]])
    xTrue = xEst
    PEst = np.eye(6)

    # history
    hxEst = xEst
    hxTrue = xTrue

    show_animation = False
    print(__file__ + " start!!")
    while k < car.loops:

        z_bar = estimator.time_propagation_update()
        F_t, W_t = estimator.time_linearization()
        sigma_bar = estimator.covariance_update()
        h_z = estimator.get_observation_model()
        H_t = estimator.observation_linearization()
        k_gain = estimator.kalman_gain_value()
        error = estimator.error_calculation(car_sensor_readout[k])
        z_hat = np.array([estimator.conditional_mean()])
        z_hat_list = np.concatenate((z_hat_list, z_hat), axis=0)
        sigma_hat = estimator.observation_update_covariance()

        hxEst = np.vstack((hxEst, z_hat))
        hxTrue = np.vstack((hxTrue, car_state[k]))


        if show_animation:
            plt.cla()
            plt.plot(np.array(hxTrue[:, 0]).flatten(),
                     np.array(hxTrue[:, 1]).flatten(), "-b")
            plt.plot(np.array(hxEst[:, 0]).flatten(),
                     np.array(hxEst[:, 1]).flatten(), "-r")
            #plot_covariance_ellipse(z_hat, sigma_hat)
            plt.ylim(0, 800)
            plt.xlim(280, 320)
            #plt.axis("equal")
            plt.grid(True)
            plt.pause(.001)

        k = k + 1




if __name__ == '__main__':
    main()

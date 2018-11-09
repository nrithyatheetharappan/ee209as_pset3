"""

Extended kalman filter plotting code

"""

import numpy as np
import math
import matplotlib.pyplot as plt
from pset3_test_test import car_simulation
from pset3_test_test import EKF
from matplotlib.animation import FuncAnimation

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
    input1 = 2
    input2 = 2
    sim_time = 6 # the car travels at 20 mm per second
    wheel_radius = 20
    wheel_base = 85
    time_step = .01
    x_i = 300
    y_i = 0
    theta_i = 0
    width = 500
    length = 1000
    car = car_simulation(wheel_radius, input1, input2, wheel_base, time_step, sim_time, x_i, y_i, theta_i, width, length)
    car_state = car.get_simulation()
    car_sensor_readout = car.get_sensor_simulation()
    estimator = EKF(input1, input2, time_step, wheel_base, wheel_radius, sim_time, x_i, y_i, theta_i, width, length)
    z_hat_list = np.zeros((1, 5))

    # State Vector [x y yaw v]'
    xEst = np.array([[x_i, y_i, theta_i, 0, 0]])
    xTrue = xEst
    PEst = np.eye(5)

    # history
    hxEst = xEst
    hxTrue = xTrue

    show_animation = False
    show_animation2 = False
    show_animation3 = True

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
            #plt.ylim(0, 700)
            #plt.xlim(280, 320)
            #plt.axis("equal")
            plt.grid(True)
            plt.pause(.009)

        k = k + 1



    xdata, ydata = hxTrue[:, 0].flatten(), hxTrue[:, 1].flatten()
    xdata2, ydata2 = hxEst[:, 0].flatten(), hxEst[:, 1].flatten()

    if show_animation2:

        fig, ax = plt.subplots()
        ax.grid()
        ln, = ax.plot([], [], '-b', animated=True)
        ln2, = ax.plot([], [], '-r', animated=True)

        def init():
            ax.set_xlim(200, 400)
            ax.set_ylim(0, 1000)
            return ln, ln2,

        def update(i):
            x = xdata[:i]
            y = ydata[:i]
            x2 = xdata2[:i]
            y2 = ydata2[:i]
            ln.set_data(x, y)
            ln2.set_data(x2, y2)
            ax.set_ylim(0, hxTrue[i][0])
            return ln, ln2,
        #
        ani = FuncAnimation(fig, update, interval=10, frames= int(car.loops) + 1,
                            init_func=init, blit=True, repeat = False)

        #ani.save('firstAni.gif', writer='imagemagick')

        plt.show()

    if show_animation3:
        #
        # setup figure
        # https://stackoverflow.com/questions/17895698/updating-the-x-axis-values-using-matplotlib-animation
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(1,1,1)
        ax1.grid()
        # set up viewing window (in this case the 25 most recent values)
        repeat_length = (np.shape(hxTrue)[0] + 1) / 100
        print(np.shape(hxTrue))
        print(repeat_length)
        ax1.set_xlim([np.amin(hxTrue[:][0]), np.amax(hxTrue[:][0])])
        #ax1.set_xlim([298, 302])
        ax1.set_ylim([0, repeat_length])

        # set up list of images for animation

        im1, = ax1.plot([], [], color=(0, 0, 1))

        def func(n):
            im1.set_xdata(xdata[:n])
            im1.set_ydata(ydata[:n])
            lim = ax1.set_ylim(0, ydata[n+1])
            if np.amin(xdata[:n+1]) > xdata[0]:
                lim2 = ax1.set_xlim(xdata[0]-.005, np.amax(xdata[:n+1]+.005))
            else:
                lim2 = ax1.set_xlim(np.amin(xdata[:n+1]-.005), np.amax(xdata[:n+1]+.005))
            return im1,

        ani1 = FuncAnimation(fig1, func, interval = 10, frames=int(car.loops), blit=False)

        plt.show()



if __name__ == '__main__':
    main()




























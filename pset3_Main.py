#all below values?

# this can go in main
if __name__ == '__main__':
    import numpy as np
    import math
    import matplotlib.pyplot as plt
    #intializations

    c1,c2,c3,c4 = 1,1,1,1

    phi_1 = 0.75
    phi_2 = 0.5
    dt = 0.2
    L = 85
    Q = np.zeros((2,2))
    Q[0][0] = c1 * np.random.normal(0,0.66) * np.random.normal(0,0.66)
    Q[1][1] = c1 * np.random.normal(0,0.66) * np.random.normal(0,0.66)
    R = np.zeros((4,4))
    R[0][0] = c1 * np.random.normal(0, .04)
    R[1][1] = c2 * np.random.normal(0, .04)
    R[2][2] = c3 * np.random.normal(0, .01)
    R[3][3] = c4 * np.random.normal(0, .01)
    r = 20


    ekf = EKF(phi_1,phi_2,dt,L,Q,R,r)
    car = car_simulation(r,phi_1,phi_2, L)
    car_trajectory = []
    estimation_trajectory = []
    covariance = []


    for i in range(0,10):
        #car_trajectory.append(call car simulation)
        #call sensor simulation
        X_val = car.get_simulation()
        car_trajectory.append((X_val[0][0],X_val[1][0]))
        #print(car.get_simulation())
        #check for time step to compute estimate?
        #print("first loop",i)

        if i % 2 == 0:
            #print("inside loop")
            U,Sigma = ekf.run_EKF()
            #print(U)
            #print(Sigma)
            estimation_trajectory.append((U[0][0],U[1][0]))
            covariance.append(Sigma)
        #print(estimation_trajectory)
    # print("car")
    # print(car_trajectory)
    # print("kalman")
    # print(estimation_trajectory)
    # print("covariance")
    # print(covariance)

    plt.plot(car_trajectory,label = 'bot', color = 'b')
    plt.plot(estimation_trajectory,label = 'kalman', color = 'g')
    plt.show()
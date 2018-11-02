
# coding: utf-8

# In[ ]:


class World:
    def __init__(self,Length,Width,radius,L):
        self.Length = Length
        self.Width = Width
        self.radius = radius
        self.L = L

        
class car_simulation:
    def __init__(self,r,phi_1,phi_2, L):
        self.theta_t_measured = 0
        self.x_t_measured = 0
        self.y_t_measured = 0
        self.X = np.zeros((6,1))
        self.r = r
        self.phi_1 = phi_1
        self.phi_2 = phi_2
        self.L = L
    def get_simulation(self):
        #change standard deviation
        #make them global? thetas?
        # get t.. t = t + something?
        w_omega_t = np.random.normal(0,1)
        w_v_t = np.random.normal(0,1)
        w_t = ((self.r*self.phi_1) - (self.r*self.phi_2))/self.L
        v_t = ((self.r*self.phi_1) + (self.r*self.phi_2))/2
        #time evolution
        self.theta_t_measured = self.theta_t_measured + ((w_t + w_omega_t) * dt)
        self.x_t__measured = self.x_t_measured + ((v_t + w_v_t) * math.cos(self.theta_t_measured) * dt)
        self.y_t_measured = self.y_t_measured + ((v_t + w_v_t) * math.sin(self.theta_t_measured) * dt)
        v_measured = v_t + w_v_t
        omega_measured = w_t + w_omega_t
        self.X = [self.x_t_measured,self.y_t_measured,v_measured,self.theta_t_measured,omega_measured,0]
        return self.X

    
class EKF(object):
    def __init__(self,d,phi,phi_1,phi_2,dt,L,Q,R):
        self.dt = dt
        #self.intial_state = intial_state
        self.d = d
        self.phi = phi
        self.phi_1 = phi_1
        self.phi_2 = phi_2
        self.X_hat = np.zeros((6,1))
        self.omega_t = ((r*phi_1) - (r*phi_2))/L
        self.v_t = ((r*phi_1) + (r*phi_2))/2
        self.X_bar = np.array((6,1))
        self.F_t = np.zeros((6,6))
        self.W_t = np.zeros((6,2))
        self.sigma_hat = np.zeros((3,3))#check
        self.H_t = np.zeros((4,6))
        self.bias = 0
        
    def time_propogation_update(self):
    # measurement_values and estimated values are numpy arrays
    #the ones with the bar, previous estimated values or prev finalized values?
        theta_t = self.X_hat[3][0]
        x_t = self.X_hat[0][0]
        y_t = self.X_hat[1][0]
        theta_t_plus_one = theta_t + self.omega_t * self.dt
        x_t_plus_one = x_t + self.v_t* math.cos(omega_t) * self.dt
        y_t_plus_one = y_t + self.v_t * math.cos(omega_t) * self.dt

        self.X_bar = [x_t_plus_one, y_t_plus_one, self.v_t,theta_t_plus_one, self.omega_t, self.bias ]
    def time_linearization(self):
        #move initializations out of the funtions
        theta_t = self.X_bar[3][0]
        self.F_t = find_F_t(self.F_t,theta_t, self.v_t, self.dt)
        
        self.W_t = find_W_t(self.W_t, theta_t, self.dt)
        #noise_matrix = some guassian
#         linearized_matrix = np.dot(F_t,estimated_state) + np.dot(W_t,noise_matrix)
#         return linearized_matrix
    def covariance_update(self):
        sigma_t_plus_one_temp = np.dot(self.F_t,self.sigma_hat)
        sigma_t_plus_one_temp = np.dot(sigma_t_plus_one_temp,self.F_t.transpose())
        #Q
        sigma_t_plus_one_temp1 = np.dot(self.W_t,self.Q)
        sigma_t_plus_one_temp1 = np.dot(sigma_t_plus_one_temp1,self.W_t.transpose())
        self.sigma_bar = sigma_t_plus_one_temp + sigma_t_plus_one_temp1
    #def observation model     
    def observation_linearization():
    
        self.H_t = find_H_t(self.H_t,self.observation,self.X_bar,landmark_values)
        #np.dot(H_t,estimated_state) + velocities? 
    def kalman_gain_value(self):
        #R = expectation of measurement noise
        #estimated state or linearizered matrix?
        #unpack values
        #use H_t from linearization?
        #H_t = np.zeros((2,3))
        #H_t = find_H_t(H_t,observation,estimated_state,landmark_values)
        inner_temp = np.dot(self.H_t,self.sigma_bar)
        inner_temp = np.dot(inner_temp,self.H_t.transpose())
        inner_temp = inner_temp + self.R
        inner_temp = np.linalg.inv(inner_temp)
        outer_temp = np.dot(self.sigma_bar,self.H_t.transpose())
        self.kalman_gain = np.dot(inner_temp, outer_temp)
        #return gain
    #def error
    def observation_update(self):
        temp_product = np.dot(self.kalman_gain,error)
        self.X_hat = self.X_bar + temp_product
        #return updated_state
    def covariance_observation_update(self):
        #take Ht out?
        inner_product = np.dot(self.kalman_gain,self.H_t)
        inner_product = np.dot(inner_product,self.sigma_bar)
        self.sigma_hat = self.sigma_bar - inner_product
        #return updated_covariance
    def run_EKF(self):
        self.time_propogation_update()
        self.time_linearization()
        self.covariance_update()
        self.observation_model()
        self.observation_linearization()
        self.kalman_gain_value()
        self.get_error()
        self.covariance_update()
        self.observation_linearization()
        return self.X_hat,self.sigma_hat

    
#all below values?

# this can go in main
import numpy as np
import math
ekf = EKF(d,phi,phi_1,phi_2,dt,L,Q,R)
car_trajectory = []
estimation_trajectory = []
for i in range(0,200):
    #car_trajectory.append(call car simulation)
    #call sensor simulation
    #check for time step to compute estimate?
    estimation_trajectory.append(ekf.run_ekf)
    


# In[ ]:


#Common functions
def find_F_t(F_t,theta_t, v_t, dt):
    F_t[0][3] = -1 * v_t * math.sin(theta_t) * dt
    F_t[1][3] =  v_t * math.cos(theta_t) * dt
    F_t[0][0],F_t[1][1],F_t[3][3],F_t[5][5] = 1,1,1,1
    return F_t

def find_W_t(W_t,theta_t, dt):
    W_t[0][0] = math.cos(theta_t) * dt
    W_t[1][0] = math.sin(theta_t) * dt
    W_t[3][1]= dt
    W_t[2][0],W_t[4][1] = 1,1
    return W_t

def find_H_t(H_t,observation,estimated_state,landmark_values):
    #unpack values
    x_t = estimated_state[0][0]
    y_t = estimated_state[2][0]
    x_l = landmark_values[0][0]
    y_l = landmark_values[1][0]
    d = observation[0][0]
    H_t[0][0] = (x_t - x_l)/d
    H_t[0][1] = (y_t - y_l)/d
    H_t[1][0]= -1 * ((y_t - y_l)/(d*d))
    H_t[1][1]= (x_t - x_l)/(d*d)
    H_t[1][2] = -1
    return H_t
    


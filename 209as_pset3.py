
# coding: utf-8

# In[30]:





# In[6]:


#IMPORTS
import numpy as np
import math
print()


# In[ ]:


#Common functions
def find_F_t(F_t,theta_t, v_t):
    F_t[1][0] = -1 * v_t * math.sin(theta_t) * dt
    F_t[2][0] =  v_t * math.cos(theta_t) * dt
    F_t[0][0],F_t[1][1],F_t[2][2] = 1,1,1
    return F_t

def find_W_t(W_t,theta_t):
    W_t[1][1] = math.cos(theta_t) * dt
    W_t[2][1] = math.sin(theta_t) * dt
    W_t[0][0]= dt
    return W_t

def find_H_t(H_t,,observation,estimated_state,landmark_values):
    #unpack values
    x_t = estimated_state[0][0]
    y_t = estimated_state[1][0]
    x_l = landmark_values[0][0]
    y_l = landmark_values[1][0]
    d = observation[0][0]
    H_t[0][0] = (x_t - x_l)/d
    H_t[0][1] = (y_t - y_l)/d
    H_t[1][0]= -1 * ((y_t - y_l)/(d*d))
    H_t[1][1]= (x_t - x_l)/(d*d)
    H_t[1][2] = -1
    return H_t
    


# In[ ]:


#TIME UPDATE BLLOCK
def time_propogation_update(measurement_values, estimated_state):
    # measurement_values and estimated values are numpy arrays
    #the ones with the bar, previous estimated values or prev finalized values?
    x_t = estimated_state[0][0]
    y_t = estimated_state[1][0]
    theta_t = estimated_state[2][0]
    
    v_t = measurement_value[0][0]
    omega_t = measurement_value[1][0]
    
    mean_value = np.array((3,1))
    
    theta_t_plus_one = theta_t + omega_t * dt
    x_t_plus_one = x_t + v_t* math.cos(omega_t) * dt
    y_t_plus_one = y_t + v_t * math.cos(omega_t) * dt
    
    mean_value = [theta_t_plus_one, x_t_plus_one, y_t_plus_one]
    return mean_value
    


# In[ ]:


#TIME LINEARIZATION
def time_linearization(estimated_state):
    #move initializations out of the funtions
    F_t = np.zeros((3,3))
    F_t = find_F_t(F_t,theta_t, v_t)
    W_t = np.zeros((3,2))
    W_t = find_W_t(W_t, theta_t)
    #noise_matrix = some guassian
    linearized_matrix = np.dot(F_t,estimated_state) + np.dot(W_t,noise_matrix)
    return linearized_matrix
    
    


# In[ ]:


#COVARIANCE UPDATE
def covariance_update(sigma_t):
    sigma_t_plus_one_temp = np.dot(F_t,sigma_t)
    sigma_t_plus_one_temp = np.dot(sigma_t_plus_one_temp,F_t.transpose())
    #Q
    sigma_t_plus_one_temp1 = np.dot(W_t,Q)
    sigma_t_plus_one_temp1 = np.dot(sigma_t_plus_one_temp1,W_t.transpose())
    sigma_t_plus_one = sigma_t_plus_one_temp + sigma_t_plus_one_temp1
    return sigma_t_plus_one


# In[ ]:


#observation model


# In[ ]:


def observation_linearization():
    H_t = np.zeros((2,3))
    H_t = find_H_t(H_t,observation,estimated_state,landmark_values)
    np.dot(H_t,estimated_state) + velocities? 


# In[ ]:


def kalman_gain(observation,sigma_t_plus_one,estimated_state,landmark_values):
    #R = expectation of measurement noise
    #estimated state or linearizered matrix?
    #unpack values
    #use H_t from linearization?
    H_t = np.zeros((2,3))
    H_t = find_H_t(H_t,observation,estimated_state,landmark_values)
    inner_temp = np.dot(H_t,sigma_t_plus_one)
    inner_temp = np.dot(inner_temp,H_t.transpose())
    inner_temp = inner_temp + R
    inner_temp = np.linalg.inv(inner_temp)
    outer_temp = np.dot(sigma_t_plus_one,H_t.transpose())
    gain = np.dot(inner_temp, outer_temp)in
    return ga


# In[ ]:


#error calculation block


# In[ ]:


#observation update
def observation_update(linearized_matrix,kalman_gain,error):
    temp_product = np.dot(kalman_gain,error)
    updated_state = linearized_matrix + temp_product
    return updated_state


# In[ ]:


#covariace observation update
def covariance_observation_update(kalman_gain, sigma_t_plus_one):
    #take Ht out?
    inner_product = np.dot(kalman_gain,H_t)
    inner_product = np.dot(inner_product,sigma_t_plus_one)
    updated_covariance = sigma_t_plus_one - inner_product
    return updated_covariance


# In[ ]:



mean_value = time_propogation_update(measurement_values, estimated_state)


# In[ ]:


#simulation block
def get_simulation(r,phi_1,phi_2):
    #change standard deviation
    #make them global? thetas?
    # get t.. t = t + something?
    w_omega_t = np.random.normal(0,1)
    w_v_t = np.random.normal(0,1)
    w_t = ((r*phi_1) - (r*phi_2))/2
    v_t = ((r*phi_1) + (r*phi_2))/2
    #time evolution
    theta_t_plus_one_measured = theta_t_measured + ((w_t + w_omega_t) * dt)
    x_t_plus_one_measured = x_t_measured + ((v_t + w_v_t) * math.cos(theta_t_measured) * dt)
    y_t_plus_one_measured = y_t_measured + ((v_t + w_v_t) * math.sin(theta_t_measured) * dt)
    return
    
        
    


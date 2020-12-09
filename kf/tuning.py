from scipy.stats import norm
import pickle
import numpy as np
import matplotlib.pyplot as plt

def fitting():    
    # model
    A = np.eye(2)
    B = np.matrix("1.5 0.1; 0.2 -0.5")

    #sensor model
    C = np.matrix("1.05 0.01; 0.01 0.9")

    # load data
    with open("../astar_path.pickle","rb") as f:
        in_dict = pickle.load(f)
        noisy_measurement = in_dict['sensor_data']
        actions = in_dict['action']
        ground_truth_states = in_dict['groundtruth']
        N = actions.shape[0]

    motion_errors = np.zeros((2,N))
    sensor_errors = np.zeros((2,N))
    for i in range(1,N):
        x_t = np.matrix(ground_truth_states[i,:]).transpose()
        x_tminus1 = np.matrix(ground_truth_states[i-1,:]).transpose()
        u_t = np.matrix(actions[i,:]).transpose()
        z_t = np.matrix(noisy_measurement[i,:]).transpose()
        
        motion_errors[:,i] = np.squeeze(x_t - A*x_tminus1 - B*u_t)
        sensor_errors[:,i] = np.squeeze(z_t - C*x_t)
    
    motion_cov=np.cov(motion_errors)
    sensor_cov=np.cov(sensor_errors)

    return motion_cov,sensor_cov

if __name__ == "__main__":
    motion_cov,sensor_cov = fitting()
    
    print "Motion cov:"
    print motion_cov
    print "Sensor cov"
    print sensor_cov
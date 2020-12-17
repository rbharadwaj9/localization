from scipy.stats import norm
import pickle
import numpy as np
import matplotlib.pyplot as plt

def fitting(filename, A, B, C):    
    # load data
    with open(filename,"rb") as f:
        in_dict = pickle.load(f)
        noisy_measurement = in_dict['sensor_data']
        actions = in_dict['action']
        ground_truth_states = in_dict['groundtruth']
        N = noisy_measurement.shape[0]

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
    sensor_cov=np.cov(in_dict['sensor_noise'].transpose())
    # sensor_cov=np.cov(sensor_errors)

    return motion_cov,sensor_cov

if __name__ == "__main__":
    motion_cov,sensor_cov = fitting("/home/boray/Intro_Algorithmic_Robot/eecs498-localization/astar_path.pickle")
    
    print "Motion cov:"
    print motion_cov
    print "Sensor cov"
    print sensor_cov

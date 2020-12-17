# File carries the sensor model based on the robot's status in the openrave environmen
import numpy as np

class Sensor:

    def __init__(self, C, cov):
        self.C = C
        self.cov = cov

    def noise_model(self, outlier_rate = 0.05, outlier_range = 0.7):
        val = np.random.multivariate_normal([0,0], self.cov).T
        val = np.reshape(val,(-1,1))

        # Outlier
        if np.random.rand(1) < outlier_rate:
            theta = np.random.uniform(0,np.pi*2)
            val = val + outlier_range*np.matrix([[np.sin(theta)],[np.cos(theta)]])
        return val

    # return ideal sensor data
    def gen_ideal(self, state):
        return self.C * state

    # generate noisy sensor data, expected (n,1) state input
    def gen_noisy(self, state):
        return self.C *state + self.noise_model()

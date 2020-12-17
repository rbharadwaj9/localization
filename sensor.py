# File carries the sensor model based on the robot's status in the openrave environmen
import numpy as np
class Sensor:

    def __init__(self, C, cov):
        self.C = C
        self.cov = cov

    def noise_model(self):
        val = np.random.multivariate_normal([0,0], self.cov).T
        val = np.reshape(val,(-1,1))
        return val

    # return ideal sensor data
    def gen_ideal(self, state):
        return self.C * state

    # generate noisy sensor data, expected (n,1) state input
    def gen_noisy(self, state):
        return self.C *state + self.noise_model()
        
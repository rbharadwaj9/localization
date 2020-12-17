# File carries the sensor model based on the robot's status in the openrave environmen
import numpy as np

class Sensor:

    def __init__(self, C):
        self.C = C

    # define sensor noise
    def noise_model(self,outlier_rate = 0.05, outlier_range = 2):
        val = np.random.multivariate_normal([0,0],[[ 1e-1, -8e-2 ],[ -8e-2,1e-1]]) # distribution
        val = np.reshape(val,(2,1))
        # Outlier
        if np.random.rand(1) < outlier_rate:
            theta = np.random.uniform(0,np.pi*2)
            # print theta
            # print [[np.sin(theta)],[np.cos(theta)]]
            val = val + 1*np.matrix([[np.sin(theta)],[np.cos(theta)]])
        return val

    def gen_single_ideal(self, x):
        return self.C * x.T

    def gen_single_noisy(self, x):
        return self.C * x.T + self.noise_model()
    
    # return ideal sensor data
    def gen_ideal(self):
        sensor_data = []
        for x in self.groundtruth:
            sensor_data.append( self.gen_single_ideal(x) )
        sensor_data = np.squeeze(sensor_data)
        sensor_data = np.array(sensor_data)
        return sensor_data

    # generate noisy sensor data
    def gen(self):
        sensor_data = []
        for x in self.groundtruth:
            sensor_data.append( self.gen_single_noisy(x) )
        sensor_data = np.squeeze(sensor_data)
        self.sensor_data = np.array(sensor_data)
        return self.sensor_data
    
    def save_data(self, filename, obj_name):
        obj_save = self.source_dict
        obj_save[obj_name] = self.sensor_data
        with open(filename,'wb') as f:
            pickle.dump(obj_save,f,protocol=pickle.HIGHEST_PROTOCOL)

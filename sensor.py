# File carries the sensor model based on the robot's status in the openrave environmen

class Sensor:

    def __init__(self, C):
        self.C = C

    def noise_model(self, outlier_rate = 0.3, outlier_range = 2):
        cov = np.array([[5e-3, -6e-5],
                        [-6e-5, 5e-3]])
        val = np.random.multivariate_normal([0,0], cov).T

        if np.random.rand(1) < outlier_rate:
            theta = np.random.uniform(0,np.pi*2)
            val += outlier_range * np.array([[np.sin(theta)], 
                                             [np.cos(theta)]])
        return val

    # return ideal sensor data
    def gen_ideal(self, ground_truth):
        sensor_data = []
        for x in ground_truth:
            sensor_data.append(self.C * x.T)
        sensor_data = np.squeeze(sensor_data)
        sensor_data = np.array(sensor_data)
        return sensor_data

    # generate noisy sensor data
    def gen_noisy(self):
        sensor_data = []
        for x in ground_truth:
            sensor_data.append(self.C * x.T + self.noise_model())
        sensor_data = np.squeeze(sensor_data)
        self.sensor_data = np.array(sensor_data)
        return self.sensor_data
    
    def save_data(self, filename, obj_name):
        obj_save = self.source_dict
        obj_save[obj_name] = self.sensor_data
        with open(filename,'wb') as f:
            pickle.dump(obj_save,f,protocol=pickle.HIGHEST_PROTOCOL)

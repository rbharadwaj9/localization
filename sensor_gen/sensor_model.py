import numpy as np
import pickle
import os

class sensor_gen:
    # initial sensor_gen object
    # C(optional): linear relationship between groundtruth and ideal measurement, default: eye(2)
    # filename: pickle file that contains groundtruth
    def __init__(self,**kwargs):
        if 'C' not in kwargs:
            self.C = np.eye(2)
        else:
            self.C = kwargs['C']

        if 'filename' not in kwargs:
            print('sensor_gen error: Require groundtruth to generate\r\n')
        else:
            self.groundtruth = self.readfromfile_pickle(kwargs['filename'])
            print "read successful, ground truth shape:",self.groundtruth.shape
            self.N = len(self.groundtruth)
        self.gen()

    # read groundtruth by pickle, and assume only first 2 columns of data are are observable(x,y)
    def readfromfile_pickle(self,filename):
        with open(filename, "rb") as f:
            groundtruth = pickle.load(f)
        return np.matrix(groundtruth)[:,0:2]

    # define sensor noise
    def noise_model(self,shape):
        return np.random.rand(shape[0],shape[1])-0.5
    
    # return ideal sensor data
    def gen_ideal(self):
        sensor_data = []
        for x in self.groundtruth:
            sensor_data.append( self.C*x.T )
        sensor_data = np.squeeze(sensor_data)
        sensor_data = np.array(sensor_data)
        return sensor_data

    # generate noisy sensor data
    def gen(self):
        sensor_data = []
        for x in self.groundtruth:
            sensor_data.append( self.C*x.T + self.noise_model((2,1)) )
        sensor_data = np.squeeze(sensor_data)
        self.sensor_data = np.array(sensor_data)
        return self.sensor_data
    
    def save_data(self,filename):
        with open(filename,'wb') as f:
            pickle.dump(self.sensor_data,f)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    groundtruth_file = 'astar_path.pickle'
    s = sensor_gen(filename=groundtruth_file)
    
    # plot difference between ideal and noisy sensor data
    plt.ion()
    plot_axes = plt.subplot(111)

    print s.sensor_data[0:s.N,0].T.shape

    plt.scatter(s.sensor_data[0:-1,0].T,s.sensor_data[0:-1,1].T,label='noisy')
    s_ideal = s.gen_ideal()
    plt.scatter(s_ideal[:,0],s_ideal[:,1],label='ideal')
    plt.legend()
    plt.show()
    raw_input('press enter')
    s.save_data('sensor_data.pickle')
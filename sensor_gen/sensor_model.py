import numpy as np
import matplotlib.pyplot as plt
import pickle
import os
from sys import argv
from scipy.stats import skewnorm

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
            self.groundtruth = self.readfromfile_pickle(kwargs['filename'],'groundtruth')
            print "read successful, ground truth shape:",self.groundtruth.shape
            self.N = len(self.groundtruth)
        self.gen()

    # read groundtruth by pickle, and assume only first 2 columns of data are are observable(x,y)
    def readfromfile_pickle(self,filename,src_objname):
        with open(filename, "rb") as f:
            self.source_dict = pickle.load(f)
        return np.matrix(self.source_dict[src_objname])[:,0:2]

    # define sensor noise
    def noise_model(self,outlier_rate = 0.05, outlier_range = 2):
        # val = np.random.multivariate_normal([0,0],[[ 2e-2, 9.99e-3 ],[9.99e-3, 1e-2]]) # distribution
        val = skewnorm.rvs(-10.,0.,0.5,2) + 0.12222620633749148
        val = np.reshape(val,(2,1))
        # Outlier
        # if np.random.rand(1) < outlier_rate:
        #     theta = np.random.uniform(0,np.pi*2)
        #     # print theta
        #     # print [[np.sin(theta)],[np.cos(theta)]]
        #     val = val + 0.5*np.array([np.sin(theta),np.cos(theta)])
        return np.reshape(val,(2,1))
    
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
            sensor_data.append( self.C*x.T + self.noise_model() )
        sensor_data = np.squeeze(sensor_data)
        self.sensor_data = np.array(sensor_data)
        return self.sensor_data
    
    def save_data(self,filename,obj_name):
        obj_save = self.source_dict
        obj_save[obj_name] = self.sensor_data
        with open(filename,'wb') as f:
            pickle.dump(obj_save,f,protocol=pickle.HIGHEST_PROTOCOL)


if __name__ == "__main__":
    try:
        groundtruth_file = argv[1]
    except IndexError:
        print "Please enter a filename of path data"
        exit(0)
    s = sensor_gen(filename=groundtruth_file,C=np.matrix('1 0;0 1'))
    
    # plot difference between ideal and noisy sensor data
    plt.ion()
    ax0 = plt.subplot(221)
    ax1 = plt.subplot(222)
    ax2 = plt.subplot(223)
    ax3 = plt.subplot(224)
    
    sample = np.zeros((2,10000))
    for i in range(10000):
        sample[:,i] = np.squeeze(s.noise_model())
    ax0.scatter(sample[0,:],sample[1,:],alpha = 0.3)
    ax2.hist(sample[0,:])
    ax3.hist(sample[1,:])

    print s.sensor_data[0:s.N,0].T.shape

    ax1.scatter(s.sensor_data[:,0].T,s.sensor_data[:,1].T,c='b',label='noisy')
    s_ideal = s.gen_ideal()
    ax1.scatter(s_ideal[:,0],s_ideal[:,1],c='k',label='ideal')
    ax1.legend()
    input_str = raw_input('save? y/n ')
    if input_str == 'y':
        s.save_data(groundtruth_file,'sensor_data')
    
    # save sensor noise samples
    with open(groundtruth_file, "rb") as f:
        source_dict = pickle.load(f)
        source_dict['sensor_noise'] = sample.transpose()
    with open(groundtruth_file, "wb") as f:
        pickle.dump(source_dict,f,protocol=pickle.HIGHEST_PROTOCOL)

import numpy as np
from sys import argv
import matplotlib.pyplot as plt
import pickle

class motion:
    def __init__(self,A,B):
        self.A = A # drift
        self.B = B # action model
    def getAction(self,xt,x0):
        return np.matmul(np.linalg.inv(self.B),(xt - np.matmul(self.A,x0)))
    def noise(self):
        p = np.random.rand(2,1)
        pos = np.greater(p,0.85,dtype=float)
        neg = np.less(p,0.15,dtype=float)
        noise = np.reshape(np.random.multivariate_normal([0,0],[[.003,.001],[.001,.001]]),(2,1)) # pos*5e-3 + neg*-5e-3
        return noise

    def getOutput(self,x0,u):
        return np.matmul(self.A,x0)+np.matmul(self.B,u)+self.noise()

if __name__ == "__main__":
    plt.ion()
    m = motion(np.eye(2),np.eye(2))
    # showcase how distribution looks like
    ax1 = plt.subplot(121)
    sample = np.zeros((2,1000))
    for i in range(100):
        sample[:,i] = np.squeeze(m.noise())
    ax1.scatter(sample[0,:],sample[1,:],alpha = 0.3)

    # read path from astar
    try:
        filename = argv[1]
    except IndexError:
        print "Please enter a filename of path data"
        exit(0)

    with open(filename, "rb") as f:
        in_dict = pickle.load(f)
    path = in_dict['path']
    
    # generate action and groundtruth with noise injected
    path = np.array(path)
    action = np.zeros((path.shape[0],2))
    groundtruth = np.zeros((path.shape[0],2))
    groundtruth[0,:] = path[0,0:2]
    for i in range(1,path.shape[0]):
        xt = np.reshape(path[i,:2].transpose(),(2,1))
        x0 = np.reshape(path[i-1,:2].transpose(),(2,1))
        # print x0
        # print xt
        u = m.getAction(xt,x0)
        # print u
        action[i,:] = np.squeeze(u)
        groundtruth[i,:] = np.squeeze(m.getOutput(x0,u))
    ax = plt.subplot(122)
    ax.plot(groundtruth[:,0],groundtruth[:,1])   
    raw_input('press continue')
    # save groundtruth and actions
    s = raw_input('save? y/n ')
    if s == 'y':
        in_dict['action'] = action
        in_dict['groundtruth'] = groundtruth
        with open(filename,'wb') as f:
            pickle.dump(in_dict,f,protocol=pickle.HIGHEST_PROTOCOL)

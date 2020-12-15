#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches  import Ellipse
import pickle
from tuning import *
from noise_model import *
from mpl_toolkits.mplot3d import Axes3D

def ParticleFilter(x,w,motion,sense,u,z,ax_before=None,ax_after=None):
    # Initialization
    m = x.shape[0]
    S = np.zeros((x.shape[0],x.shape[1]+1))
    Xt = np.zeros_like(x)
    Wt = np.zeros(x.shape[0])

    # Updating particles & calculating weights
    for i in range(m):
        x0 = np.matrix(x[i,:]).T
        # print 'u'
        # print u
        # print 'x0'
        # print x0
        xt = motion.getOutput(u,x0)
        P_z_given_x = sense.p(z,xt)

        weight = P_z_given_x * w[i]
        S[i,:] = np.concatenate((xt.T,np.matrix(weight)),axis=1)
    S[:,2] = S[:,2]/np.sum(S[:,2])
    

    C = np.matrix("1.05 0.01; 0.01 0.9")
    x_sensor = np.linalg.inv(C)*z  
    # print "---------"
    # print 'x_sensor'
    # print x_sensor
    # print sense.p(z,x_sensor)
    # print 'x_max'
    # print S[np.argmax(S[:,2]),:]
    # print np.max(S[:,2])
    # print "weight"
    # print np.sum(S[:,2])
    # print np.average(S[:,2])

    # Plot weights of particles
    try:
        ax_before.scatter(list(S[:,0]),list(S[:,1]),list(S[:,2]))
        ax_before.plot([x_sensor[0,0],x_sensor[0,0]],[x_sensor[1,0],x_sensor[1,0]],[0,np.max(S[:,2])],c='r')
        ax_before.set_xlabel('x')
        ax_before.set_ylabel('y')
        ax_before.set_zlabel('weight')
    except:
        pass

    # Re-sampling
    # low-variance resampling
    r = np.random.rand(1)/m
    n = 0
    for i in range(m):
        weight_pick = r + float(i)/float(m)
        for j in range(n,m):
            sum = np.sum(S[:j+1,2])
            if sum >= weight_pick:
                Xt[i,:] = S[j,0:2]
                Wt[i] = S[j,2]
                n = j
                break
    Wt = Wt/np.sum(Wt)
    
    # print "n"
    # print n
    # print "Weight average"
    # print np.average(Wt)
    try:
        ax_after.scatter(list(Xt[:,0]),list(Xt[:,1]),list(Wt))
        ax_after.set_xlabel('x')
        ax_after.set_ylabel('y')
        ax_after.set_zlabel('weight')
    except:
        pass

    # determine max likelihood
    # weighted average
    sum = np.zeros((2))
    for i in range(m):
        sum = sum + Xt[i,:]*Wt[i]
    x_max = sum/np.sum(Wt)
    # x_max = np.average(Xt,axis = 0)#Xt[np.argmax(Wt),:]
    try:
        ax_after.plot([x_max[0],x_max[0]],[x_max[1],x_max[1]],[0,np.max(S[:,2])],c='r')
    except:
        pass

    return Xt,Wt, x_max

class motion:

    def __init__(self,noise):
        self.noise = noise
        self.A = np.eye(2)
        self.B = np.matrix("1.5 0.1; 0.2 -0.5")
    
    def getOutput(self,u,x):
        return self.A*x+self.B*u+self.noise.getRandom()
    def p(self,xnew,x,u):
        return self.noise.p(xnew - self.A*x+self.B*u)

class sensing:
    def __init__(self,noise):
        self.noise = noise
    def p(self,z,xt):
        C = np.matrix("1.05 0.01; 0.01 0.9")
        error = z-C*xt
        p = self.noise.p(error)
        # print "error"
        # print error
        # print "p"
        # print p
        return p


def main():

    x_range = [-3, 3]
    y_range = [-3, 3]

    # Initiailize
    N0 = 20000
    X = np.zeros((N0,2))
    X[:,0] = np.random.uniform(x_range[0],x_range[1],N0)
    X[:,1] = np.random.uniform(y_range[0],y_range[1],N0)
    W = np.ones(N0)/N0

    #load in the data
    PIK = "kfdata.dat"
    with open(PIK, "rb") as f:
        noisy_measurement,actions,ground_truth_states,N = pickle.load(f)
    
    #Ready to plot
    ax = []
    ax.append(plt.subplot(121))
    ax.append(plt.subplot(122))
    # ax.append(plt.subplot(312, projection='3d'))
    # ax.append(plt.subplot(313, projection='3d'))
    
    motion_cov, sensor_cov = fitting()

    sn = sensor_noise(np.matrix('0;0'),sensor_cov)
    an = action_noise(np.matrix('0;0'),motion_cov)

    m = motion(an)
    s = sensing(sn)
    N = 100
    estimate = np.zeros((N,2))
    
    ax[0].scatter(list(X[:,0]),list(X[:,1]),c='b',label='particles')
    # plt.pause(0.05)
    # raw_input("press to continue")
    
    for i in range(1,N):

        ax[0].cla()
        ax[1].cla()
        # ax[2].cla()
        # ax[3].cla()

        z = np.matrix(noisy_measurement[:,i]).transpose() #current x
        u = np.matrix(actions[:,i]).transpose()           #current u
        #run Particle Filter
        X,W,xt = ParticleFilter(X,W,m,s,u,z) #,ax[2],ax[3]
        
        estimate[i,:] = xt
        


        x_true = [ground_truth_states[0,i],ground_truth_states[1,i]]
        title = "Step " + str(i)
        plt.title(title)
        ax[0].scatter(list(X[:,0]),list(X[:,1]),c='b',label='particles')
        ax[0].scatter(float(xt[0]),float(xt[1]),c='r',label='max likelihood')
        ax[0].plot(list(estimate[:i+1,0]),list(estimate[:i+1,1]),'r')
        ax[0].plot(ground_truth_states[0,0:i+1],ground_truth_states[1,0:i+1],c='k',label='ground truth')
        ax[0].scatter(ground_truth_states[0,0:i+1],ground_truth_states[1,0:i+1],c='k',label='ground truth')
        ax[0].set_xlabel('x')
        ax[0].set_ylabel('y')
        ax[0].legend()
        ax[1].hist(W)
        # ax[2].plot([x_true[0],x_true[0]],[x_true[1],x_true[1]],[0,np.max(W)],c='k')
        # ax[3].plot([x_true[0],x_true[0]],[x_true[1],x_true[1]],[0,np.max(W)],c='k')
        
        plt.pause(1)
        # raw_input("press to continue")
    state_errors = estimate.transpose()[:,0:N] - ground_truth_states[:,0:N]
    total_error=np.sum(np.linalg.norm(state_errors, axis=0))
    print "Total Error: %f"%total_error
    raw_input("press to continue")

if __name__ == "__main__":
    main()
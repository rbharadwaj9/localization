#!/usr/bin/env python
import numpy as np
from scipy.stats import skewnorm

class ParticleFilter:
    def __init__(self,N,init_range,random_ratio,noisetype=None,R=None,Q=None):
        self.X = np.zeros((N,len(init_range)))
        self.W = np.ones(N)/N
        self.N = N
        self.random_ratio = random_ratio
        self.init_range = init_range
        self.noisetype = noisetype

        if noisetype == 'skewnorm':
            self.Q = Q
            s = np.linspace(skewnorm.ppf(0.01, Q[0], Q[1], Q[2]),skewnorm.ppf(0.99, Q[0], Q[1], Q[2]), 100)
            y = skewnorm.pdf(s, Q[0], Q[1], Q[2])
            self.sensingNoiseMode = s[np.argmax(y)]

        for i,random_range in enumerate(init_range):
            self.X[:,i] = np.random.uniform(random_range[0],random_range[1],N)
    def motion_random(self,R):
        rand = np.random.multivariate_normal([0,0],R)
        return np.reshape(rand,(-1,1))
    def sensing_pdf(self,C,Q,z,x):
        if self.noisetype == 'multivariate_normal':
            k = len(z)
            e = z-np.matmul(C,x)
            p1 = np.power((2*np.pi),-k/2)*np.power(np.linalg.det(Q),-0.5)
            p2 = np.exp(-0.5*(e).T*np.linalg.inv(Q)*(e))
            e_ = e*(np.linalg.norm(e)-0.5)
            p2_ = np.exp(-0.5*(e_).T*np.linalg.inv(Q)*(e_))
            return float(p1*p2) + float(p1*p2_)

        if self.noisetype == 'skewnorm':
            e = z-np.matmul(C,x) + self.sensingNoiseMode
            return np.linalg.norm(skewnorm.pdf(e, self.Q[0], self.Q[1], self.Q[2]))
    
    def filter(self,mu,Sigma,z,u,A,B,C,Q,R):
        # Initialization
        m = self.N
        S = np.zeros((self.X.shape[0],self.X.shape[1]+1))
        Xt = np.zeros((self.X.shape[0],self.X.shape[1]))
        Wt = np.zeros(self.X.shape[0])

        # Updating particles & calculating weights
        for i in range(m):
            x0 = np.matrix(self.X[i,:]).T
            xt = np.matmul(A,x0) + np.matmul(B,u) + self.motion_random(R)
            P_z_given_x = self.sensing_pdf(C,Q,z,xt)
            weight = P_z_given_x * self.W[i]
            S[i,:] = np.concatenate((xt.T,np.matrix(weight)),axis=1)
        S[:,2] = S[:,2]/np.sum(S[:,2])

        # Re-sampling
        # low-variance resampling
        r = float(np.random.rand(1)/float(m))
        # print r
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

        # ax.scatter(Xt[:,0],Xt[:,1],Wt,c='b')
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')

        # determine max likelihood
        # weighted average
        sum = np.zeros((2))
        for i in range(m):
            sum = sum + Xt[i,:]*Wt[i]
        x_max = sum/np.sum(Wt)
        
        # inject randomness
        # w_min = np.quantile(Wt,0.10)
        # indeces = []
        # while len(indeces) < round(self.N*self.random_ratio):
        #     n = np.random.randint(0,self.N)
        #     if n not in indeces:
        #         indeces.append(n)
        #         for i,random_range in enumerate(self.init_range):
        #             Xt[n,i] = np.random.uniform(random_range[0],random_range[1],1)
                
        #         Wt[n] = self.sensing_pdf(C,Q,z,np.matrix(Xt[n,:]).T)*w_min

        cov = np.cov(Xt.transpose())

        self.X = Xt
        self.W = Wt

        return x_max, cov

import matplotlib.pyplot as plt
import pickle
from tuning import fitting
from mpl_toolkits.mplot3d import Axes3D

def main():
    # pf = ParticleFilter(500,[[-4.,4.],[-1.5,4.]],0.1,'multivariate_normal') #,Q=[-13.63,0.0056,0.517]
    pf = ParticleFilter(500,[[-4.,4.],[-1.5,4.]],0.1,'skewnorm', Q=[-10,0,0.5])

    PIK = "data/env2_hwk3_skewnorm.pickle"
    with open(PIK,"r") as f:
        in_dict = pickle.load(f)
    actions = in_dict["action"].transpose()
    path = in_dict["groundtruth"].transpose()
    noisy_measurement = in_dict["sensor_data"].transpose()
    A = np.matrix('1 0 ; 0 1')
    B = np.matrix(np.eye(2))
    C = np.matrix('1 0; 0 1')
    start = np.linalg.inv(C)*np.matrix(noisy_measurement[:, 0]).T
    R,Q = fitting(PIK,A,B,C)

    print "noise cov"
    print R
    print Q

    N = path.shape[1]

    estimate_states = np.zeros((2,N))
    estimate_states[:,0] = np.squeeze(np.array(start))
    plt.ion()
    fig = plt.subplot(131)
    ax  = plt.subplot(132, projection = '3d')
    ax1 = plt.subplot(133)
    for i in range(1,N):
        
        fig.cla()
        ax.cla()
        ax1.cla()
        
        fig.scatter(pf.X[:,0],pf.X[:,1],c='b')
        fig.plot(path[0,:i],path[1,:i],c = 'k')
        fig.plot(estimate_states[0,:i],estimate_states[1,:i],c='r')
        
        ax.scatter(pf.X[:,0],pf.X[:,1],pf.W,c='b')
        ax.plot([path[0,i],path[0,i]],[path[1,i],path[1,i]],[0,np.max(pf.W)],c='k')

        ax1.hist(pf.W)

        z = np.matrix(noisy_measurement[:,i]).transpose()
        fig.scatter(z[0,0],z[1,0],c='g')
        u = np.matrix(actions[:,i]).transpose()

        xh, cov = pf.filter(None,None,z,u,A,B,C,Q,R)
        estimate_states[:,i] = np.squeeze(np.array(xh))
        plt.pause(0.5)
    
    state_errors = estimate_states[1:N,:] - path[1:N,:]
    total_error=np.sum(np.linalg.norm(state_errors, axis=0))
    print "Total Error: %f"%total_error
    fig.plot(path[0,:],path[1,:],c = 'k')
    fig.plot(estimate_states[0,:],estimate_states[1,:],c='r')
    plt.show()
    raw_input("press")

if __name__ == "__main__":
    main()

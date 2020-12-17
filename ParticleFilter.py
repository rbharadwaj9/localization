#!/usr/bin/env python
import numpy as np

class ParticleFilter:
    def __init__(self,N,init_range):
        self.X = np.zeros((N,len(init_range)))
        self.W = np.ones(N)/N
        self.N = N
        for i,range in enumerate(init_range):
            self.X[:,i] = np.random.uniform(range[0],range[1],N)
    def motion_random(self,R):
        rand = np.random.multivariate_normal([0,0],R)
        return np.reshape(rand,(-1,1))
    def sensing_pdf(self,C,Q,z,x):
        k = len(z)
        e = z-C*x
        return (np.power((2*np.pi),-k/2)*np.power(np.linalg.det(Q),-0.5)*np.exp(-0.5*e.T*np.linalg.inv(Q)*e))
    
    def filter(self,mu,Sigma,z,u,A,B,C,Q,R):
        # Initialization
        m = self.N
        S = np.zeros((self.X.shape[0],self.X.shape[1]+1))
        Xt = np.zeros_like(self.X)
        Wt = np.zeros(self.X.shape[0])

        # Updating particles & calculating weights
        for i in range(m):
            x0 = np.matrix(self.X[i,:]).T
            xt = A*x0 + B*u + self.motion_random(R)
            P_z_given_x = self.sensing_pdf(C,Q,z,xt)
            weight = P_z_given_x * self.W[i]
            S[i,:] = np.concatenate((xt.T,np.matrix(weight)),axis=1)
        S[:,2] = S[:,2]/np.sum(S[:,2])

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

        # determine max likelihood
        # weighted average
        sum = np.zeros((2))
        for i in range(m):
            sum = sum + Xt[i,:]*Wt[i]
        x_max = sum/np.sum(Wt)
        
        # average
        # x_max = np.average(Xt,axis = 0)#Xt[np.argmax(Wt),:]

        cov = np.cov(Xt.transpose())

        self.X = Xt
        self.W = Wt

        return x_max, cov
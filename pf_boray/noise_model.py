import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import multivariate_normal

class action_noise:
    # assume multivariate normal dist
    # mu: kx1 np.array
    # cov: kxk np.array
    def __init__(self,mu,cov):
        self.mu = mu
        self.cov = cov
        self.k = mu.shape[0]
        print self.k
    # p(x)
    def p(self,x):
        return (np.power((2*np.pi),-self.k/2)*np.power(np.linalg.det(self.cov),-0.5)*np.exp(-0.5*(x-self.mu).T*np.linalg.inv(self.cov)*(x-self.mu)))
    def getRandom(self):
        return np.matrix(np.random.multivariate_normal(np.array(self.mu).flatten(),self.cov)).T

class sensor_noise:
    # assume multivariate normal dist
    # mu: kx1 np.array
    # cov: kxk np.array
    def __init__(self,mu,cov):
        self.mu = mu
        self.cov = cov
        self.k = mu.shape[0]
    # p(x)
    def p(self,x):
        # rv = multivariate_normal(mean=self.mu,cov=self.mu,allow_singular=True)
        # return rv.pdf(tuple(x))
        return (np.power((2*np.pi),-self.k/2)*np.power(np.linalg.det(self.cov),-0.5)*np.exp(-0.5*(x-self.mu).T*np.linalg.inv(self.cov)*(x-self.mu)))
    def getRandom(self):
        # rv = multivariate_normal(mean=self.mu,cov=self.mu,allow_singular=True)
        # return rv.rvs()
        return np.matrix(np.random.multivariate_normal(np.array(self.mu).flatten(),self.cov)).T

if __name__ == '__main__':
    plt.ion()
    fig = plt.figure()
    ax1 = fig.add_subplot(211, projection='3d')
    ax2 = fig.add_subplot(212, projection='3d')

    mu = np.matrix('0;0')
    cov = np.matrix('2.50696845e-03 1.79957758e-05;1.79957758e-05 2.51063277e-03')
    a_noise = action_noise(mu,cov)

    mu = np.matrix('0;0')
    cov = np.matrix('0.04869528 -0.0058636;-0.0058636   1.01216104')
    s_noise = sensor_noise(mu,cov)

    scale_x = 5
    scale_y = 10
    noise = a_noise
    x = (np.random.random(1000)-0.5)*scale_x
    y = (np.random.random(1000)-0.5)*scale_y
    z = np.zeros(1000)
    samples = np.zeros((2,1000))
    samples_p = np.zeros(1000)
    for i in range(1000):
        X = np.matrix([x[i],y[i]]).T
        z[i] = noise.p(X)
        samples[:,i] = np.squeeze(np.array(noise.getRandom()))
        samples_p[i] = noise.p(np.matrix(samples[:,i]).T)
    print np.max(noise.p((0,0)))

    print multivariate_normal.pdf(x=[0,0],cov=np.matrix('2.50696845e-03 1.79957758e-05;1.79957758e-05 2.51063277e-03'))
    
    ax1.scatter(x,y,z)
    ax2.scatter(samples[0,:],samples[1,:],samples_p)
    plt.show()

    raw_input("press to continue")
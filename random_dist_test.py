from scipy.stats import skewnorm
import matplotlib.pyplot as plt
import numpy as np

def m0(a):
    delta = a/np.sqrt(1+a**2)
    mu_z = np.sqrt(2/np.pi) * delta
    r1 = (4-np.pi)/2 * ((delta*np.sqrt(2/np.pi))**3)   /   ((1-2*(delta**2)/np.pi)**1.5)
    sigma_z = np.sqrt(1-mu_z**2)
    return mu_z - r1*sigma_z/2 - np.sign(a)/2 * np.exp(-2*np.pi/np.abs(a))

a = -10
loc = 0
w = 0.5

median = skewnorm.median(a,loc,w)
s = skewnorm.rvs(a,loc,w,1000)
mode = loc + w*m0(a)

plt.ion()
ax = plt.subplot(111)

print(type(s))
print(s.shape)


x = np.linspace(skewnorm.ppf(0.01,a,loc,w),skewnorm.ppf(0.99, a,loc,w), 100)
y = skewnorm.pdf(x, a,loc,w)
mode = x[np.argmax(y)]

x = x - mode
s = s - mode


ax.plot(x, y,'r-', lw=5, alpha=0.6, label='skewnorm pdf')
ax.axvline(x=0,color='y',label='mode')
ax.hist(s,density = True)
ax.set_xlabel('noise')
ax.set_ylabel('Number of samples/PDF')
ax.set_title('Skewnormal histogram, loc = %.1f, w = %.1f, a = %.1f'%(a,loc,w))
text = 'Shift all noise by mode = %.3f' % mode
props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
ax.text(0.05, 0.8, text, transform=ax.transAxes, fontsize=12,
        verticalalignment='top', bbox=props)
ax.legend()
plt.show()

print 'mode: ',mode

model = skewnorm.fit(s)
print model

raw_input('press')
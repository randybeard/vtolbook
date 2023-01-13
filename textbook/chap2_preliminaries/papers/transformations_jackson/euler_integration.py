import numpy as np
import matplotlib.pyplot as plt
from math import exp
from matplotlib import rc
from scipy.linalg import expm
rc('text', usetex=True)
a = 1

def dx(x, u):
    return np.array([[
        u[0] * np.sin(x[2,0]),
        u[0] * np.cos(x[2,0]),
        u[1]
    ]]).T

def R(theta):
    return np.array([[np.cos(theta), np.sin(theta)],
                     [-np.sin(theta), np.cos(theta)]])

def f(x0, u, t):
    T = np.zeros((3,3))
    T[:2,:2] = R(x0[2,0])
    T[:2,2] = x0[:2,0]
    T[2,2] = 1

    v = u[0,0]
    w = u[1,0]

    delta_skew = np.array([[0, -w, v],
                           [w, 0,  0],
                           [0, 0,  0]])
    Tout = T.dot(expm(delta_skew * t))
    return np.array([[Tout[0,2],
                      Tout[1,2],
                      np.arcsin(Tout[0,0])]]).T



x0 = np.array([[0, 0, 0]]).T
u = np.array([[0.5, 1.0]]).T
t0 = 0
tf = 1.0

plt.figure()
time =np.linspace(t0, tf, 1000)
x = np.hstack([f(x0, u, t) for t in time])
plt.plot(x[1,:], x[0,:], label="$e^t$")
print(x)
for k in range(6):
    i = k*k
    if i < 2: continue
    dt = (tf - t0)/float(i)
    time = np.linspace(t0, tf, i)
    x = np.zeros((3,i))
    x[:,0,None] = x0
    # print(dt)
    # print(x)
    # print(time)
    for j, t in enumerate(time):
        if j == 0:
            x[:,j,None] = x0
            continue
        x[:,j,None] = dx(x[:,j-1,None], u)*dt + x[:,j-1,None]
    plt.plot(x[0,:], x[1,:], label="{0:.3f}".format(dt))

plt.legend()
plt.axis('equal')
plt.show()

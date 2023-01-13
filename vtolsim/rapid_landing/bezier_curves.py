#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

def B_i_n(i, n, u):
    return np.math.factorial(n)/(np.math.factorial(i)*np.math.factorial(n-i)) \
        * (u**i) * (1-u)**(n-i)

def C(u, p, n):
    c = 0
    for i in range(n + 1):
        c += B_i_n(i, n, u) * np.array(np.array([p[i]]).T)
    return c


OP_1 = [[0, 0],
        [0, 2],
        [1, 3],
        [2, 3],
        [3, 1],
        [1, -1]]
OP_2 = [[0, 0],
        [1.5, 1],
        [-.5, 1],
        [1, 0]]

control_points = OP_1
# control_points = [[0, 0],
#                   [1, 5],
#                   [1, 5],
#                   [2, 0]]

n = len(control_points) - 1

u_array = np.linspace(0, 1, 200)
c_array = []
for u in u_array:
    c_array.append(C(u, control_points, n))

c_array = np.array(c_array)
control_points = np.array(control_points)

plt.plot(c_array[:,0], c_array[:,1])
plt.plot(control_points[:,0], control_points[:,1], )
plt.show()



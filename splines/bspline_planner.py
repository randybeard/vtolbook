"""
messing with b-splines
        2/18/21 - RWB
"""
import numpy as np
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

order = 3  # order of spline polynomials
knots = [0, 0, 0, 0, 1, 2, 3, 3, 3, 3]  # n+order+1 knot points
ctrlpts = np.array([  # n-3D control points
    [0., 0., 0.],
    [1., 0., 0.],
    [0, 1, 0.],
    [1., 1., 0],
    [1., 1., 1.],
    [1., 1., 2.],
    ])
spl = BSpline(knots, ctrlpts, order)  # create spline
t = np.linspace(0, 3, 100)
spline_fine = spl(t)

# plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(ctrlpts[:, 0], ctrlpts[:, 1], ctrlpts[:, 2],
        '-o', label='definition nodes')
ax.plot(spline_fine[:, 0], spline_fine[:, 1], spline_fine[:, 2],
        'b', label='spline')
ax.legend()
ax.set_xlabel('x', fontsize=16, rotation=0)
ax.set_ylabel('y', fontsize=16, rotation=0)
ax.set_zlabel('z', fontsize=16, rotation=0)

plt.show()

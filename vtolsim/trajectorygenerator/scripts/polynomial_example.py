#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

from trajectory import Trajectory
from polynomial_trajectory_generator import PolynomialTrajectoryGenerator
from basis.standard_basis_member import StandardBasisMember
import trajectory_plotter


"""
General Discussion:

Polynomial trajectories consist of a sequence of points, referred to
as knot points, connected by a polynomial function that is continuous
up to some derivative.

To create a trajectory, it is important to think about the concept of
degrees of freedom. Degrees of freedom refers to the number of attributes
you can specify and have them all achieved exactly. These attributes
include things like derivatives such as position, velocity, and acceleration
or things like continuity at a derivative (without specifying the value).
Degrees of freedom can be calculated as
(num_segments * num_bases - num_attributes).

To increase the number of degrees of freedom without reducing the number
of attributes you specify, you have two options: increase the number of
segments, or increase the number of basis functions used for the
polynomial. Since increasing the number of segments will change your
trajectory, and since the number of segments is typically higher than the
number of basis functions, it makes sense to increase num_bases.
A typical choice is num_bases = 10.

The number of segments is entirely dependent on your application. Typical
values are probably in the 10 - 20 range, but fewer or more should be fine.
Note, however that as you produce fewer, the number of attributes you can
specify starts going down - especially since the start and end points
typically have more attributes (think takeoff/landing).
"""

"""
# Example 1
"""

"""
We'll start out by defining the attributes of a trajectory
"""
num_segments = 10
num_bases = 7

"""Ensure continuous up to (and including) 3rd derivative"""
continuity_derivative = 3
"""Optimize smoothness of 3rd derivative"""
smoothness_derivative = 3

"""
We want to set the value of every knot point,
the velocity of the first and last,
and the acceleration of the last.

Note that there are num_segments+1 knot points.
"""
knot_constraint_structure = [
    {'deriv': 0,
        'knot_list': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]},
    {'deriv': 1,
        'knot_list': [0, 1, 10]},
    {'deriv': 2,
        'knot_list': [10]},
]

"""
Finally, we need to specify the type of basis that we will use.
"""
basis_class = StandardBasisMember

"""
Now we use these attributes to define a PolynomialTrajectoryMember.
Notice that we haven't specified the value of any of the points
along the trajectory. This will come next.
"""
gen = PolynomialTrajectoryGenerator(
    num_segments,
    num_bases,
    basis_class,
    continuity_derivative,
    smoothness_derivative,
    knot_constraint_structure
)

"""
The generator can now produce any trajectory that has the above
attributes.

To produce a trajectory, we define a knot_constraint_values list.
It has a similar layout to the knot_constraint_structure list, but
instead of specifying which knot points we want to constrain, we
specify the value to constrain those knot points to.
The knot points correspond to the ones in the
knot_constraint_structure list.

We'll define a separate knot_constraint_values list for x, y, and z
"""
kcv_x = [
    # position - all knot points
    {'deriv': 0,
        'value_list': [0., 5., 10., 15., 20.,
                       25., 20., 15., 10., 5., 0.]},
    # velocity - 0, 1, and 10 knot points
    {'deriv': 1,
        'value_list': [0., 5., 0.]},
    # acceleration - 10th knot point
    {'deriv': 2,
        'value_list': [0.]}
]

kcv_y = [
    # position - all knot points
    {'deriv': 0,
        'value_list': [0., 5., 10., 5., 0.,
                       -5., -10., -5., 0., -5., 0.]},
    # velocity - 0, 1, and 10 knot points
    {'deriv': 1,
        'value_list': [0., 5., 0.]},
    # acceleration - 10th knot point
    {'deriv': 2,
        'value_list': [0.]}
]

kcv_z = [
    # position - all knot points
    {'deriv': 0,
        'value_list': [0., -5., -10., -15., -15.,
                       -15., -10., -10., -5., 2., 0.]},
    # velocity - 0, 1, and 10 knot points
    {'deriv': 1,
        'value_list': [0., -5., .1]},
    # acceleration - 10th knot point
    {'deriv': 2,
        'value_list': [0.]}
]

"""
Now we can put these into a list and generate a trajectory.
"""
kcv_list = [kcv_x, kcv_y, kcv_z]
ex1_traj = gen.generate_trajectory(kcv_list)


trajectory_plotter.plotVsTime(ex1_traj, 0, 0, 10)
trajectory_plotter.plotVsTime(ex1_traj, 1, 0, 10)
trajectory_plotter.plotVsTime(ex1_traj, 2, 0, 10)

# plot n, e, d in 3D
trajectory_plotter.plot3D(ex1_traj, 0, [0, 1, 2], 0, 10)

plt.show()

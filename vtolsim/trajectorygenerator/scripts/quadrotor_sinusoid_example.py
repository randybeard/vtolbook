#!/usr/bin/env python3
# Example of using a sinusoidal trajectory for a quadrotor

import matplotlib.pyplot as plt
import numpy as np

from trajectory import Trajectory
from sinusoidal_trajectory_member import SinusoidalTrajectoryMember
from linear_trajectory_member import LinearTrajectoryMember
import trajectory_plotter

"""
For controlling a quadrotor, our trajectory will consist of four members:
Pn, Pe, Pd - the position of the quadrotor in NED coordinates
Ppsi - the rotation angle of the quadrotor about the inertial k axis

A trajectory based on sinusoids is easy to define and differentiate,
and it is possible to define it as a loop, making it easy to allow a
simulation to run as long as you want.
The basic idea is that each trajectory member will have the following form:
    Px(tau)   = offset + scale * sin(tau * (2*pi/period) + phase)
The kth derivative of this is simply
    Px_k(tau) = scale * (2*pi/period)^k * sin(tau * (2*pi/period) + phase + k*pi/2)

Notice the use of tau in these expressions. This is to allow time scaling
without changing the shape of the overall trajectory.
We let tau = time_scale * t.
Increasing the time scale will cause the trajectory to become more
aggressive, while decreasing the time scale will cause the trajectory
to become less aggressive.

To define a sinusoidal trajectory, we use the SinusoidalTrajectory class.
It takes four arrays of parameters as inputs:
R_offset - The offset of each trajectory member.
R_scale - The scale of each trajectory member.
R_period - The period of each trajectory member. When time_scale = 1.0, this is in seconds.
        A period of 0 is handled by making the frequency zero - the value is constant
R_phase - The phase shift of each trajectory member.
The length of these arrays defines the desired number of trajectory members.
So, for a quadrotor, each should be of length four.
"""

"""
Example 1 -
5m radius pringle shaped orbit
at 10m altitude
with zero yaw
and a period of 60 sec.
"""

# SinusoidalTrajectoryMember(offset, scale, period, phase)
pringle_Pn_traj_memb = SinusoidalTrajectoryMember(0., 5., 60., 0.)
pringle_Pe_traj_memb = SinusoidalTrajectoryMember(0., 5., 30., 0.)
pringle_Pd_traj_memb = SinusoidalTrajectoryMember(-30., 5., 60., np.pi/2)

# LinearTrajectoryMember(offset, slope)
pringle_Ppsi_traj_memb = LinearTrajectoryMember(0., 0.)

quad_pringle_membs = [
    pringle_Pn_traj_memb,
    pringle_Pe_traj_memb,
    pringle_Pd_traj_memb,
    pringle_Ppsi_traj_memb
    ]

quad_pringle_traj = Trajectory(quad_pringle_membs, time_scale=1.0)

# plot the trajectory and its first two derivatives
trajectory_plotter.plotVsTime(quad_pringle_traj, 0, 0, 60)
trajectory_plotter.plotVsTime(quad_pringle_traj, 1, 0, 60)
trajectory_plotter.plotVsTime(quad_pringle_traj, 2, 0, 60)

# plot n, e, d in 3D
trajectory_plotter.plot3D(quad_pringle_traj, 0, [0, 1, 2], 0, 60)

"""
For controlling a quadrotor, we need to know up to the second derivative
of the position and the first derivative of the yaw.
To get these, we can use the Trajectory.evalUpToKr function,
which evaluates the trajectory and it's derivatives up to and including kr.

In the differential flatness literature, the vector of the trajectory members
and their derivatives is known as the flat flag, so that's what it is
referred to as here.

The rows of quad_circle_flag correspond to each trajectory member, 
and the columns correspond to the kth derivative, starting from 0.
"""

t = 1.0
kr = 2
quad_pringle_flag = quad_pringle_traj.evalUpToKr(t, kr)
print("quad_pringle_flag = ", quad_pringle_flag)

plt.show()

"""
Example 2
5m radius circular orbit in i-j plane
at 10m altitude
with yaw pointing to center of circle
and a period of 30 seconds.

For this example, we are going to create each TrajectoryMember individually
and then use them to instantiate a Trajectory object. This allows us to define
Pn and Pe as sinusoids and Pd and Ppsi as linear functions.
"""

circle_Pn_traj_memb = SinusoidalTrajectoryMember(0., 5., 30., 0.)
circle_Pe_traj_memb = SinusoidalTrajectoryMember(0., 5., 30., np.pi/2)
circle_Pd_traj_memb = LinearTrajectoryMember(-10., 0.)

# The trajectory starts at Pn=0, Pe=5, so psi should be -pi/2 at this point.
# psi is then a linear function of time, doing a full rotation of -2*pi
# every 30 seconds. This does not wrap.
circle_Ppsi_traj_memb = LinearTrajectoryMember(-np.pi/2, -2*np.pi/30.)

quad_circle_membs = [
    circle_Pn_traj_memb,
    circle_Pe_traj_memb,
    circle_Pd_traj_memb,
    circle_Ppsi_traj_memb
    ]

quad_circle_traj = Trajectory(quad_circle_membs, time_scale=1.0)

trajectory_plotter.plotVsTime(quad_circle_traj, 0, 0, 60)
trajectory_plotter.plotVsTime(quad_circle_traj, 1, 0, 60)
trajectory_plotter.plotVsTime(quad_circle_traj, 2, 0, 60)


# plot n, e, d in 3D
trajectory_plotter.plot3D(quad_circle_traj, 0, [0, 1, 2], 0, 60)

quad_circle_flag = quad_circle_traj.evalUpToKr(t, kr)
print("quad_circle_flag = ", quad_circle_flag)

plt.show()

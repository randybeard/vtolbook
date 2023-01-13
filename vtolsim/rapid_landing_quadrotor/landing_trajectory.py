"""
simple landing trajectory
        2/17/22 - RWB
        3/11/22 - RWB
"""
import numpy as np
from math import ceil
from scipy.interpolate import BSpline
from scipy.linalg import norm
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import sys
sys.path.append('..')
import parameters.convergence_parameters as VTOL
from message_types.msg_autopilot import msgAutopilot

class TrajectoryPlanner:
    def __init__(self, ts_control):
        self.Ts = ts_control
        self.commands = msgAutopilot()

    def renew(self, p0, v0, pf, vf, start_time, end_time):
        self.tf = end_time
        self.pos_spl = landingTrajectory(p0, v0, pf, vf, start_time, self.tf, num_segments=1)
        self.vel_spl = self.pos_spl.derivative()
        self.accel_spl = self.vel_spl.derivative()
        self.jerk_spl = self.accel_spl.derivative()
        self.commands.pos = self.pos_spl(start_time).reshape((3,1))
        self.commands.vel = self.vel_spl(start_time).reshape((3,1))
        self.commands.accel = self.accel_spl(start_time).reshape((3,1))
        self.commands.heading = 0.
        #plotSpline(self.pos_spl)
        return self.commands

    def update(self, time):
        if time <= self.tf:
            self.commands.pos = self.pos_spl(time).reshape((3,1))
            self.commands.vel = self.vel_spl(time).reshape((3,1))
            self.commands.accel = self.accel_spl(time).reshape((3,1))
            self.commands.heading = 0.
        else:
            self.commands.pos = self.pos_spl(self.tf).reshape((3,1))
            self.commands.vel = self.vel_spl(self.tf).reshape((3,1))
            self.commands.accel = self.accel_spl(self.tf).reshape((3,1))
            self.commands.heading = 0.
        return self.commands

def uniformKnots(t0, tf, order, num_segments):
    knots = np.concatenate((
        t0 * np.ones(order),
        np.concatenate((
            np.arange(t0, tf, (tf-t0)/(num_segments)),
            tf * np.ones(order+1)),
            axis=0)
        ), axis=0)
    return knots

def controlPointsPosVel(p0, v0, pf, vf, knots, order, num_segments):
    N = num_segments + order
    ctrl_pts = np.zeros((N, 3))
    # set first two control points to specify initial position and velocity
    ctrl_pts[0, :] = p0.T[0,:]
    ctrl_pts[1,:] = p0.T + ( (knots[order + 1] - knots[1]) / order ) * v0.T
    # intermediate control points in straight line between p0 and pf
    for i in range(2, N-2):
        alpha = (i-1) / (num_segments+1)
        ctrl_pts[i,:] = (1-alpha) * p0.T + alpha * pf.T
    # set last two control points to specify initial position and velocity
    ctrl_pts[N-2,:] = pf.T - (knots[order + N-1]-knots[N-1])/order * vf.T
    ctrl_pts[N-1, :] = pf.T[0,:]
    return ctrl_pts

def landingTrajectory(p0, v0, pf, vf, t0, tf, num_segments=1):
    order = 3  # order of the spline
    knots = uniformKnots(t0, tf, order, num_segments)
    N = num_segments + order
    ctrl_pts = controlPointsPosVel(p0, v0, pf, vf, knots, order, num_segments)
    spl = BSpline(t=knots, c=ctrl_pts, k=order)
    # x0 = np.reshape(ctrl_pts[2:N-2,:], (1,3*(N-4)))
    # res = minimize(cost, x0, args=spl, method='nelder-mead', )
    # spl.c[2:-2, :] = np.reshape(res.x, (N-4, 3))
    return spl


def cost(x, spl):
    t0 = spl.t[0]  # first knot is t0
    tf = spl.t[-1]  # last knot is tf
    N = ceil((tf - t0)/0.1)  # number of points in time vector so spacing is 0.1
    time = np.linspace(t0, tf, N)  # time vector
    m, n = np.shape(spl.c)
    num_intermediate_cpts = m-4
    spl.c[2:-2,:] = np.reshape(x, (num_intermediate_cpts, 3))
    vel, accel, jerk= splineDerivatives(spl, time)
    J = 1.0 * max(accel) + 1.0 * max(jerk)
    return J


def splineMagnitude(spl, time):
    mag = []
    for i in range(0, len(time)):
        mag.append(norm(spl(time[i])))
    return mag


def splineDerivatives(spl, time):
    spl_dot = spl.derivative()
    spl_ddot = spl_dot.derivative()
    spl_dddot = spl_ddot.derivative()
    vel = splineMagnitude(spl_dot, time)
    accel = splineMagnitude(spl_ddot, time)
    jerk = splineMagnitude(spl_dddot, time)
    return vel, accel, jerk


def plotSpline(spl):
    t0 = spl.t[0]  # first knot is t0
    tf = spl.t[-1]  # last knot is tf
    N = ceil((tf - t0)/0.01)  # number of points in time vector so spacing is 0.01
    t = np.linspace(t0, tf, N)  # time vector
    position = spl(t)
    vel, accel, jerk = splineDerivatives(spl, t)
    # 3D trajectory plot
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    # plot control points (convert YX(-Z) -> NED)
    ax.plot(spl.c[:, 1], spl.c[:, 0], -spl.c[:, 2],
            '-o', label='control points')
    # plot spline (convert YX(-Z) -> NED)
    ax.plot(position[:, 1], position[:, 0], -position[:, 2],
            'b', label='spline')
    ax.legend()
    ax.set_xlabel('x', fontsize=16, rotation=0)
    ax.set_ylabel('y', fontsize=16, rotation=0)
    ax.set_zlabel('z', fontsize=16, rotation=0)
    ax.set_xlim3d([-10, 10])
    # velocity plot
    fig = plt.figure(2)
    ax = fig.add_subplot(311)
    ax.plot(t, vel)
    ax.set_xlabel('time', fontsize=14, rotation=0)
    ax.set_ylabel('velocity', fontsize=14)
    # acceleration plot
    ax = fig.add_subplot(312)
    ax.plot(t, accel)
    ax.set_xlabel('time', fontsize=14, rotation=0)
    ax.set_ylabel('acceleration', fontsize=14)
    # jerk plot
    ax = fig.add_subplot(313)
    ax.plot(t, jerk, label='jerk')
    ax.set_xlabel('time', fontsize=14, rotation=0)
    ax.set_ylabel('jerk', fontsize=14)
    plt.show()


if __name__ == "__main__":
    # initial and final positions and velocities (NED)
    p0 = np.array([[100, 0, -10]]).T
    v0 = np.array([[-10, 0, 0]]).T
    pf = np.array([[0, 0, 0]]).T
    vf = np.array([[0, 0, 1]]).T
    # initial and final time
    t0 = 0
    tf = 5
    spl = landingTrajectory(p0, v0, pf, vf, t0, tf, num_segments=8)
    plotSpline(spl)


#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plotVsTime(traj, k, t0, tf, tsteps=100, fig=None, ax=None, labels=None):
    """
    Plot the kth derivative of each member of the trajectory between t0 and tf.
    """

    ev, tvec = evalVector(traj, k, t0, tf, tsteps)
    if ax is None or fig is None:
        fig, ax = plt.subplots(traj.Nr, 1)

    fig.suptitle("Derivative {}".format(k))

    if traj.Nr > 1:
        for ell in range(traj.Nr):
            ax[ell].plot(tvec, ev[ell, :])
            if labels is None:
                ax[ell].set_ylabel("P{}".format(ell))
            else:
                ax[ell].set_ylabel(labels[ell])
            ax[ell].grid(True)
        ax[-1].set_xlabel("Time")
    else:
        ax.plot(tvec, ev[0,:])
        ax.set_ylabel("P{}".format(0))
        ax.set_xlabel("Time")
        ax.grid(True)

    return fig, ax


def plot3D(traj, k, ell_vec, t0, tf, tsteps=100):
    """
    Plot the kth derivative of three members between t0 and tf.
    ell_vec specifies the three desired members, it must be 3 long.
    """

    fig = plt.figure()
    ev, tvec = evalVector(traj, k, t0, tf, tsteps)
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(
        ev[ell_vec[0], :],
        ev[ell_vec[1], :],
        ev[ell_vec[2], :]
        )

    fig.suptitle("Derivative {} of P{}, P{}, and P{}".format(
        k, ell_vec[0], ell_vec[1], ell_vec[2]))

    ax.set_xlabel("P{}".format(ell_vec[0]))
    ax.set_ylabel("P{}".format(ell_vec[1]))
    ax.set_zlabel("P{}".format(ell_vec[2]))

    return fig, ax

def evalFullTraj(traj, k, tsteps=100):
    t0 = 0.
    tf = traj.members[0].total_time
    return evalVector(traj, k, t0, tf, tsteps=tsteps)


def evalVector(traj, k, t0, tf, tsteps=100):
    """
    Evaulate the kth derivative of each member of
    the trajectory between t0 and tf.
    """
    ev = np.zeros((traj.Nr, tsteps))
    tvec = np.linspace(t0, tf, tsteps, endpoint=True)

    for tk in range(tsteps):
        t = tvec[tk]

        ev[:, tk] = traj.eval(t, k)

    return ev, tvec

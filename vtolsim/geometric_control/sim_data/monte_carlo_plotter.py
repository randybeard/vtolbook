import numpy as np

from matplotlib import rc
rc('text', usetex=True)
rc('text.latex', preamble=r'\usepackage{amsmath}')
rc('font', size=8)
rc('figure', figsize=(3.5,2.25))
rc('lines', linewidth=.8)
rc('mathtext', fontset='cm')
rc('font', family='STIXGeneral')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig_prefix = "MonteCarlo"
fig_filetype = ".png"

def plot_monte_carlo(fname):

    data = np.load(fname, allow_pickle=True)

    trajectories = data["trajectories"]
    error_avg = data["error_avg"]
    error_max = data["error_max"]
    time_avg = data["time_avg"]
    time_max = data["time_max"]

    plot_monte_carlo_trajectories(trajectories)

    plot_monte_carlo_error(error_avg, error_max)

    plot_monte_carlo_time(time_avg, time_max)


    plt.show()

def plot_monte_carlo_trajectories(trajectories):
    fig, ax = plt.subplots(2,1, figsize=(6,4), gridspec_kw={"height_ratios": (2.2,1)})
    plt.subplots_adjust(right=.98, hspace=0.25, left=.08, bottom=.09, top=.99)

    for i in range(trajectories.shape[0]):
        traj = trajectories[i,:,:].T
        ax[0].plot(traj[0,:], traj[1,:])
        ax[1].plot(traj[0,:], traj[2,:])

    xlim = ax[0].get_xlim()
    ylim = ax[0].get_ylim()
    ax[0].set_xticks(np.concatenate([
        np.arange(0,int(xlim[1]+2),5),
        np.arange(0,int(xlim[0]-2),-5)
    ]))
    ax[0].set_yticks(np.concatenate([
        np.arange(0,int(ylim[1]+2),5),
        np.arange(0,int(ylim[0]-2),-5)
    ]))
    ax[0].set_aspect("equal")
    ax[0].set_xlabel(r"North (m)")
    ax[0].set_ylabel(r"East (m)")
    ax[0].grid(True)

    ax[1].set_ylabel(r"Altitude (m)")
    ax[1].set_xlabel(r"North (m)")
    ax[1].set_xticks(np.concatenate([
        np.arange(0,int(xlim[1]+2),5),
        np.arange(0,int(xlim[0]-2),-5)
    ]))
    ax[1].grid(True)

    fig.savefig(fig_prefix + "_trajectories" + fig_filetype, dpi=600)

def plot_monte_carlo_error(avg_errors, max_errors):

    fig, ax = plt.subplots(1,2, figsize=(6,3))
    plt.subplots_adjust(right=.98, left=.1, bottom=.15, top=.98)
    ax[0].hist(avg_errors, edgecolor="k")
    ax[0].set_xlabel(r"Average Position Error (m)")
    ax[1].hist(max_errors, edgecolor="k")
    ax[1].set_xlabel(r"Maximum Position Error (m)")

    ylim0 = ax[0].get_ylim()
    ylim1 = ax[1].get_ylim()
    ylim_max = max(ylim0[1], ylim1[1])
    ax[0].set_ylim((0, ylim_max))
    ax[1].set_ylim((0, ylim_max))

    ax[0].set_ylabel(r"Count")

    fig.savefig(fig_prefix + "_errors" + fig_filetype, dpi=600)

def plot_monte_carlo_time(avg_times, max_times):

    fig, ax = plt.subplots(1,2, figsize=(6,3))
    plt.subplots_adjust(right=.98, left=.1, bottom=.15, top=.98)
    ax[0].hist(avg_times**-1, edgecolor="k") # frequency
    ax[0].set_xlabel(r"Average Computation Rate (Hz)")
    ax[1].hist(max_times**-1, edgecolor="k") # frequency
    ax[1].set_xlabel(r"Minimum Computation Rate (Hz)")

    ylim0 = ax[0].get_ylim()
    ylim1 = ax[1].get_ylim()
    ylim_max = max(ylim0[1], ylim1[1])
    ax[0].set_ylim((0, ylim_max))
    ax[1].set_ylim((0, ylim_max))

    ax[0].set_ylabel(r"Count")

    fig.savefig(fig_prefix + "_times" + fig_filetype, dpi=600)

plot_monte_carlo("MonteCarlo_27Mar2021_10:34:22.npz")

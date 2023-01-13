import numpy as np

from matplotlib import rc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys
sys.path.append("../..")
from tools.rotations import Quaternion2Euler

file_type = ".pdf"
# file_type = ".png"

thesis_fmt = True

if thesis_fmt:
    rc('text', usetex=True)
    rc('text.latex', preamble=r'\usepackage{amsmath}')
    rc('font', size=10)
    rc('figure', figsize=(5,4))
    rc('lines', linewidth=1)
    rc('mathtext', fontset='cm')
    rc('font', family='STIXGeneral')
else:
    rc('text', usetex=True)
    rc('text.latex', preamble=r'\usepackage{amsmath}')
    rc('font', size=6)
    rc('figure', figsize=(3.5,2.25))
    rc('lines', linewidth=.8)
    rc('mathtext', fontset='cm')
    rc('font', family='STIXGeneral')

line_formats = ["-", "--", "-.", ":", "--", "-.", ":"]

def plot_optimized_zthrust_comparison(filenames, labels, figure_suffix):

    # read in data
    desired_positions = []
    true_positions = []
    desired_velocities = []

    times = []
    desired_quats = []
    true_quats = []
    true_thrusts = []

    for fn in filenames:

        data = np.load(fn, allow_pickle=True)

        times.append(data['time'])

        desired_state = data['desired_state']
        desired_positions.append(desired_state[:,0:3])
        desired_velocities.append(desired_state[:,3:6])
        desired_quats.append(desired_state[:,6:10])

        true_state = data['true_state']
        true_positions.append(true_state[:,0:3])
        true_quats.append(true_state[:,6:10])

        true_thrust_torque = data['true_thrust_torque']
        true_thrusts.append(true_thrust_torque[:,0:2])


    # position and yaw
    if thesis_fmt:
        fig, ax = plt.subplots(4,1, figsize=(6,4))
        plt.subplots_adjust(right=.73, hspace=0.1, left=.14, bottom=.15, top=.95)
    else:
        fig, ax = plt.subplots(4,1)
        plt.subplots_adjust(right=.84, hspace=0.1, left=.15, bottom=.15, top=.95)
    plot_position(
        desired_positions,
        true_positions,
        labels + [r"Ref."],
        times + [times[0]],
        ax[0:3]
    )

    plot_yaw_angle(
        desired_quats,
        true_quats,
        labels + [r"Ref."],
        times + [times[0]],
        ax=ax[3]
    )
    ax[3].set_xlabel(r"Time (s)")
    ax[3].grid(True)
    plt.savefig(f"Position_{figure_suffix}" + file_type, dpi=600)


    if thesis_fmt:
        fig, ax = plt.subplots(3,1)
        plt.subplots_adjust(right=.75, hspace=0.1, left=.14, bottom=.15, top=.95)
    else:
        fig, ax = plt.subplots(3,1)
        plt.subplots_adjust(right=.86, hspace=0.1, left=.15, bottom=.15, top=.95)
    # position error
    plot_position_error(
        desired_positions,
        true_positions,
        labels,
        times,
        ax=ax[0]
    )
    ax[0].set_xticklabels([])
    ax[0].grid(True)

    # thrust magnitude
    plot_thrust_magnitude(
        true_thrusts,
        labels,
        times,
        ax=ax[1]
    )
    ax[1].set_xticklabels([])
    ax[1].grid(True)

    # pitch angle
    plot_pitch_angle(
        true_quats,
        labels,
        times,
        ax=ax[2]
    )
    ax[2].set_xlabel("Time (s)")
    ax[2].grid(True)

    plt.savefig(f"Comparison_{figure_suffix}" + file_type, dpi=600)

    fig, ax = plot_3D_position(desired_positions[0], desired_velocities[0])
    fig.savefig(f"Traj3D_{figure_suffix}" + file_type, dpi=600)

    plt.show()

def plot_position(
        desired_positions,
        positions,
        labels,
        times,
        ax):

    for i in range(len(positions)):
        ax[0].plot(times[i], positions[i][:,0], "-", label=labels[i], color=f"C{i}")
        ax[1].plot(times[i], positions[i][:,1], "-", label=labels[i], color=f"C{i}")
        ax[2].plot(times[i], -positions[i][:,2], "-", label=labels[i], color=f"C{i}")

        ax[0].plot(times[i], desired_positions[i][:,0], ":", color=f"C{i}" , label= labels[i] + r" Ref.")
        ax[1].plot(times[i], desired_positions[i][:,1], ":", color=f"C{i}",  label= labels[i] + r" Ref.")
        ax[2].plot(times[i], -desired_positions[i][:,2], ":", color=f"C{i}", label= labels[i] + r" Ref.")

    ax[0].set_ylabel(r"North (m)")
    ax[0].set_xticklabels([])
    ax[0].grid(True)
    if thesis_fmt:
        ax[0].legend(loc="upper right", bbox_to_anchor=(1.45, 1.0))
    else:
        ax[0].legend(loc="upper right", bbox_to_anchor=(1.25, 1.32), borderpad=.2, labelspacing=.2, fontsize=7)

    ax[1].set_ylabel(r"East (m)")
    ax[1].set_xticklabels([])
    ax[1].grid(True)

    ax[2].set_ylabel(r"Alt. (m)")
    ax[2].grid(True)
    ax[2].set_xticklabels([])


def plot_position_error(
        desired_positions,
        true_positions,
        labels,
        times,
        ax=None):

    if ax is None:
        fig, ax = plt.subplots(1,1)

    for i in range(len(true_positions)):
        dp = desired_positions[i]
        tp = true_positions[i]
        perr = np.linalg.norm(
            dp - tp,
            axis=1
        )

        ax.plot(times[i], perr, line_formats[i], label=labels[i], color=f"C{i}")

        average_perr = np.mean(perr)
        print(f"{labels[i]} average position error = {average_perr}")


    ax.set_ylabel("Position\nerror (m)")
    if thesis_fmt:
        ax.legend(loc="upper right", bbox_to_anchor=(1.42, 1.0))
    else:
        ax.legend(loc="upper right", bbox_to_anchor=(1.2, 1.2), borderpad=.2, labelspacing=.2, fontsize=7)

def plot_thrust_magnitude(
        true_thrusts,
        labels,
        times,
        ax=None):

    if ax is None:
        fig, ax = plt.subplots(1,1)

    for i in range(len(true_thrusts)):
        tmag = np.linalg.norm(
            true_thrusts[i],
            axis=1
        )

        ax.plot(times[i], tmag, line_formats[i], label=labels[i], color=f"C{i}")

        average_thrust = np.mean(tmag)
        print(f"{labels[i]} average thrust = {average_thrust}")


    ax.set_ylabel(r"$\|$Thrust$\|$ (N)")

def quat_array_to_euler(quat_array):

    euler_array = np.zeros((quat_array.shape[0], 3))
    for i in range(quat_array.shape[0]):
        quati = quat_array[i,:]

        phi_i, theta_i, psi_i = Quaternion2Euler(quati)
        euler_array[i,0] = phi_i
        euler_array[i,1] = theta_i
        euler_array[i,2] = psi_i

    return euler_array

def plot_pitch_angle(
        quats,
        labels,
        times,
        ax=None):

    if ax is  None:
        fig, ax = plt.subplots(1,1)

    for i in range(len(quats)):
        ea = quat_array_to_euler(quats[i])

        ax.plot(times[i], np.rad2deg(ea[:,1]), line_formats[i], label=labels[i], color=f"C{i}")

    ax.set_ylabel("Pitch (deg)")

def plot_yaw_angle(
        desired_quats,
        quats,
        labels,
        times,
        ax=None):

    if ax is  None:
        fig, ax = plt.subplots(1,1)

    for i in range(len(quats)):
        ea = quat_array_to_euler(quats[i])
        dea = quat_array_to_euler(desired_quats[i])

        ax.plot(times[i], np.rad2deg(ea[:,2]), "-", label=r"True " + labels[i], color=f"C{i}")
        ax.plot(times[i], np.rad2deg(dea[:,2]), ":", label=r"Des. " + labels[i], color=f"C{i}")

    deg_yticks = np.array([-90, -45, 0, 45, 90])
    ylim = ax.get_ylim()

    ax.set_yticks(deg_yticks[np.logical_and(deg_yticks>=ylim[0], deg_yticks<=ylim[1])])
    ax.set_ylabel("Yaw (deg)")

def plot_3D_position(position, velocities):

    fig = plt.figure(figsize=(3.5,1.5))

    ax = fig.add_subplot(111, projection='3d')

    vel = np.linalg.norm(velocities, axis=1)
    print(f"Max velocity: {np.max(vel)}")

    ax.plot(
        position[:,0],
        position[:,1],
        -position[:,2]
        )

    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")

    ax.set_box_aspect((np.ptp(position[:,0]), np.ptp(position[:,1]), np.ptp(-position[:,2])))

    ax.view_init(20, -80)

    # ax.view_init()

    return fig, ax


## Thesis Spline Comparison Pitch Thrust Plots
plot_optimized_zthrust_comparison(
    [
    "PitchOptimizedSampled_BLENDED_2_AirSimPolyTCLNOpt_27Mar2021_10:04:22.npz",
    "PitchOptimizedSampled_BLENDED_2_AirSimPolyTCLOpt_27Mar2021_10:01:41.npz",
    "PitchOptimizedSampled_BLENDED_2_AirSimBSplineTCL_27Mar2021_10:07:23.npz",
    ],
    [r"Uniform", r"Optimized", r"B-spline"],
"SplineCompareTCL")

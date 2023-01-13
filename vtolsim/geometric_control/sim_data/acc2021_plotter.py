import numpy as np

from matplotlib import rc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys
sys.path.append("../..")
from tools.rotations import Quaternion2Euler

file_type = ".pdf"
# file_type = ".png"

thesis_fmt = False

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
    desired_thrusts = []

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

        desired_thrust_torque = data['desired_thrust_torque']
        desired_thrusts.append(desired_thrust_torque[:,0:2])


    # position and yaw
    if thesis_fmt:
        fig, ax = plt.subplots(4,1)
        plt.subplots_adjust(right=.80, hspace=0.1, left=.11, bottom=.15, top=.95)
    else:
        fig, ax = plt.subplots(4,1)
        plt.subplots_adjust(right=.84, hspace=0.1, left=.15, bottom=.15, top=.95)

    plot_position(
        true_positions + [desired_positions[0]],
        labels + [r"Ref."],
        times + [times[0]],
        ax[0:3]
    )

    plot_yaw_angle(
        true_quats + [desired_quats[0]],
        labels + [r"Ref."],
        times + [times[0]],
        ax=ax[3]
    )
    ax[3].set_xlabel(r"Time (s)")
    ax[3].grid(True)
    plt.savefig(f"Position_{figure_suffix}" + file_type, dpi=600)


    if thesis_fmt:
        fig, ax = plt.subplots(3,1)
        plt.subplots_adjust(right=.81, hspace=0.1, left=.12, bottom=.15, top=.95)
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


    # Thrust Error
    if thesis_fmt:
        fig, ax = plt.subplots(3,1)
        plt.subplots_adjust(right=.81, hspace=0.1, left=.12, bottom=.15, top=.95)
    else:
        fig, ax = plt.subplots(3,1)
        plt.subplots_adjust(right=.86, hspace=0.1, left=.15, bottom=.15, top=.95)

    plot_thrust_error(
        true_thrusts,
        desired_thrusts,
        labels,
        times,
        ax=ax
    )

    fig.savefig(f"Thrust_Error_{figure_suffix}" + file_type, dpi=600)

    plt.show()

def plot_position(
        positions,
        labels,
        times,
        ax):

    for i in range(len(positions)):
        ax[0].plot(times[i], positions[i][:,0], line_formats[i], label=labels[i])
        ax[1].plot(times[i], positions[i][:,1], line_formats[i], label=labels[i])
        ax[2].plot(times[i], -positions[i][:,2], line_formats[i], label=labels[i])

    ax[0].set_ylabel(r"North (m)")
    ax[0].set_xticklabels([])
    ax[0].grid(True)
    if thesis_fmt:
        ax[0].legend(loc="upper right", bbox_to_anchor=(1.27, 1.0))
    else:
        ax[0].legend(loc="upper right", bbox_to_anchor=(1.25, 1.32), borderpad=.2, labelspacing=.2, fontsize=7)
        ax[0].set_zorder(5)

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

        ax.plot(times[i], perr, line_formats[i], label=labels[i])

        average_perr = np.mean(perr)
        print(f"{labels[i]} average position error = {average_perr}")


    ax.set_ylabel("Position\nerror (m)")
    if thesis_fmt:
        ax.legend(loc="upper right", bbox_to_anchor=(1.27, 1.0))
    else:
        ax.legend(loc="upper right", bbox_to_anchor=(1.2, 1.2), borderpad=.2, labelspacing=.2, fontsize=7)
        ax.set_zorder(5)

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

        ax.plot(times[i], tmag, line_formats[i], label=labels[i])

        average_thrust = np.mean(tmag)
        print(f"{labels[i]} average thrust = {average_thrust}")


    ax.set_ylabel(r"$\|$Thrust$\|$ (N)")

def plot_thrust_error(
        true_thrusts,
        desired_thrusts,
        labels,
        times,
        ax=None):

    if ax is None:
        fig, ax = plt.subplots(3,1)

    for i in range(len(true_thrusts)):
        t_err = desired_thrusts[i] - true_thrusts[i] 
        t_err_mag = np.linalg.norm(t_err, axis=1)

        ax[0].plot(times[i], t_err[:,0], line_formats[i], label=labels[i])
        ax[1].plot(times[i], t_err[:,1], line_formats[i], label=labels[i])
        ax[2].plot(times[i], t_err_mag, line_formats[i], label=labels[i])

        average_thrust_error = np.mean(t_err_mag)
        print(f"{labels[i]} average thrust error = {average_thrust_error}")

    ax[0].set_ylabel(r"$T_{x,d/b}$ (N)")
    ax[1].set_ylabel(r"$T_{z,d/b}$ (N)")
    ax[2].set_ylabel(r"$\|\boldsymbol{T}_{d/b}\|$ (N)")
    ax[2].set_xlabel(r"Time (s)")

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

        ax.plot(times[i], np.rad2deg(ea[:,1]), line_formats[i], label=labels[i])

    ax.set_ylabel("Pitch (deg)")

def plot_yaw_angle(
        quats,
        labels,
        times,
        ax=None):

    if ax is  None:
        fig, ax = plt.subplots(1,1)

    for i in range(len(quats)):
        ea = quat_array_to_euler(quats[i])

        ax.plot(times[i], np.rad2deg(ea[:,2]), line_formats[i], label=labels[i])

    ax.set_yticks([0, 45, 90])
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


# ACC 2021 Plots
# plot_optimized_zthrust_comparison(["PitchOptimized_PolyTCL_25Sep2020_10:13:23.npz", "ZThrust_PolyTCL_25Sep2020_10:16:43.npz"],
# [r"$\theta, \boldsymbol{T}$ optimized", r"$T_x = 0$"],
# "ACC_PolyTCL")

# CDC 2021 Pitch Thrust Plots
plot_optimized_zthrust_comparison(
    ["PitchOptimizedOptimizer_BLENDED_2_ACCPolyTCL_29Apr2021_10:42:30.npz",
    "PitchOptimizedZThrust_BLENDED_2_ACCPolyTCL_29Apr2021_10:33:35.npz",
    "PitchOptimizedSampled_SMALL_ANGLE_ACCPolyTCL_29Apr2021_10:28:56.npz",
    "PitchOptimizedSampled_FLAT_PLATE_1_ACCPolyTCL_29Apr2021_10:30:33.npz",
    "PitchOptimizedSampled_BLENDED_2_ACCPolyTCL_29Apr2021_10:31:40.npz",
    ],
    [r"N Opt.", r"$\angle \boldsymbol{F}_d^d$", r"SA", r"FP1", r"B2"],
"CDC_PolyTCL")

## Thesis AoA Model Pitch Thrust Plots
# plot_optimized_zthrust_comparison(
#     ["ZThrust_ACCPolyTCL_02Mar2021_14:57:22.npz",
#     "PitchOptimizedSampled_SMALL_ANGLE_ACCPolyTCL_02Mar2021_15:00:09.npz",
#     "PitchOptimizedSampled_FLAT_PLATE_1_ACCPolyTCL_02Mar2021_15:00:58.npz",
#     "PitchOptimizedSampled_BLENDED_2_ACCPolyTCL_02Mar2021_15:01:46.npz",
#     ],
#     [r"$\angle \boldsymbol{F}_d^d$", r"SA", r"FP1", r"B2"],
# "PolyTCL_thesis")

## Thesis

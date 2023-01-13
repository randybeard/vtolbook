"""
Plot the result of using the optimal pitch routine
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
sys.path.append('..')

thesis_fmt = False # switch between thesis format (big) and paper format (small)
file_type = ".pdf"
# file_type = ".png"

if thesis_fmt:
    rc('text', usetex=True)
    rc('text.latex', preamble=r'\usepackage{amsmath}')
    rc('font', size=10)
    rc('figure', figsize=(6,4))
    rc('lines', linewidth=1)
    rc('mathtext', fontset='cm')
    rc('font', family='STIXGeneral')
else:
    rc('text', usetex=True)
    rc('text.latex', preamble=r'\usepackage{amsmath}')
    rc('font', size=6)
    rc('figure', figsize=(3.5,2.0))
    rc('lines', linewidth=.8)
    rc('mathtext', fontset='cm')
    rc('font', family='STIXGeneral')



from geometric_control.optimal_pitch import *
import parameters.convergence_parameters as VTOL

def plot_constant_F_V_changing_gamma(save=False, nsamples=200):
    # Same force, same velocity, changing gamma
    F_d_d = np.array([0., 0., -VTOL.mass*VTOL.gravity])
    Va = 3.
    gamma = np.arange(np.deg2rad(-90), np.deg2rad(180), .01)
    nsteps = len(gamma)
    v_d_d = Va * np.array([np.cos(gamma), np.zeros(nsteps), -np.sin(gamma)])

    SA_T_array = np.zeros((2,nsteps))
    SA_theta_opt_array = np.zeros(nsteps)
    FP1_T_array = np.zeros((2,nsteps))
    FP1_theta_opt_array = np.zeros(nsteps)
    B2_T_array = np.zeros((2,nsteps))
    B2_theta_opt_array = np.zeros(nsteps)

    SA_theta_opt_prev = None
    FP1_theta_opt_prev = None
    B2_theta_opt_prev = None

    T_z_array = np.zeros((2,nsteps))
    theta_z_array = np.zeros(nsteps)

    for i in range(nsteps):
        v_d_d_i = v_d_d[:,i]

        # Small Angle
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d_i, F_d_d, previous_theta=SA_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.SMALL_ANGLE)
        SA_theta_opt_prev = theta_opt_i
        SA_T_array[:,i] = T_i
        SA_theta_opt_array[i] = theta_opt_i

        # Flat Plate 1
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d_i, F_d_d, previous_theta=FP1_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.FLAT_PLATE_1)
        FP1_theta_opt_prev = theta_opt_i
        FP1_T_array[:,i] = T_i
        FP1_theta_opt_array[i] = theta_opt_i

        # Blended 2
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d_i, F_d_d, previous_theta=B2_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.BLENDED_2)
        B2_theta_opt_prev = theta_opt_i
        B2_T_array[:,i] = T_i
        B2_theta_opt_array[i] = theta_opt_i

        # ZThrust, Blended 2
        theta_z_i, T_z_i = find_pitch_thrust_ZThrust(v_d_d_i, F_d_d, model=AERO_TYPE.BLENDED_2)
        T_z_array[:,i] = T_z_i
        theta_z_array[i] = theta_z_i

    fig, ax = plot_general_thrust_pitch(
        np.rad2deg(gamma),
        [SA_T_array, FP1_T_array, B2_T_array, T_z_array],
        [SA_theta_opt_array, FP1_theta_opt_array, B2_theta_opt_array, theta_z_array],
        r"$\gamma$ (deg)",
        [r"SA, Opt", r"FP1, Opt", r"B2, Opt", r"B2, $\angle \boldsymbol{F}_d^d$"],
        xticks=[-90, -45, 0, 45, 90, 135, 180])
    if save:
        fig.savefig("sim_data/constant_F_V_changing_gamma" + file_type, dpi=600)

def plot_general_thrust_pitch(xarray, thrustlist, thetalist, xlabel, legendlabels, xticks=None):
    # plot thrust and pitch
    if thesis_fmt:
        fig, ax = plt.subplots(4,1, sharex=True)
        plt.subplots_adjust(right=.77, hspace=0.1, left=.12, bottom=.12, top=.95)
    else:
        fig, ax = plt.subplots(4,1, sharex=True)
        plt.subplots_adjust(right=.815, hspace=0.1, left=.17, bottom=.16, top=.95)

    linetypelist=["-", "--", "-.", ":"]

    for i in range(len(thrustlist)):
        ax[0].plot(xarray, thrustlist[i][0,:], linetypelist[i])
        ax[1].plot(xarray, thrustlist[i][1,:], linetypelist[i])
        ax[2].plot(xarray, np.linalg.norm(thrustlist[i], axis=0), linetypelist[i])
        ax[3].plot(xarray, np.rad2deg(thetalist[i]), linetypelist[i])

    ax[0].set_ylabel(r"$T_x$ (N)")
    ax[0].grid(True)
    if thesis_fmt:
        ax[0].legend(legendlabels, loc="upper right", bbox_to_anchor=(1.33, 1.2))
        ax[0].set_zorder(5)
    else:
        ax[0].legend(legendlabels, loc="upper right", bbox_to_anchor=(1.30, 1.32), borderpad=.2, labelspacing=.2, fontsize=7)
        ax[0].set_zorder(5)
    if xticks is not None:
        ax[0].set_xticks(xticks)

    ax[1].set_ylabel(r"$T_z$ (N)")
    ax[1].grid(True)

    ax[2].set_ylabel(r"$\|T\|$ (N)")
    ax[2].grid(True)

    ax[3].set_ylabel(r"$\theta^\star$ (deg)")
    ax[3].grid(True)
    ax[3].set_xlabel(xlabel)

    return fig, ax

def plot_constant_V_gamma_changing_F(save=False, nsamples=200):
    # changing force angle, same velocity, same gamma
    F_const = np.array([0., 0., -VTOL.mass*VTOL.gravity])
    F_x = np.arange(-VTOL.mass*VTOL.gravity, VTOL.mass*VTOL.gravity, .01)
    Va = 3.
    gamma = 0.
    v_d_d = Va * np.array([np.cos(gamma), 0., -np.sin(gamma)])

    nsteps = len(F_x)

    SA_T_array = np.zeros((2,nsteps))
    SA_theta_opt_array = np.zeros(nsteps)
    FP1_T_array = np.zeros((2,nsteps))
    FP1_theta_opt_array = np.zeros(nsteps)
    B2_T_array = np.zeros((2,nsteps))
    B2_theta_opt_array = np.zeros(nsteps)

    SA_theta_opt_prev = None
    FP1_theta_opt_prev = None
    B2_theta_opt_prev = None

    T_z_array = np.zeros((2,nsteps))
    theta_z_array = np.zeros(nsteps)

    for i in range(nsteps):
        F_d_d_i = F_const + np.array([F_x[i], 0., 0.])

        # Small Angle
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d, F_d_d_i, previous_theta=SA_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.SMALL_ANGLE)
        SA_theta_opt_prev = theta_opt_i
        SA_T_array[:,i] = T_i
        SA_theta_opt_array[i] = theta_opt_i

        # Flat Plate 1
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d, F_d_d_i, previous_theta=FP1_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.FLAT_PLATE_1)
        FP1_theta_opt_prev = theta_opt_i
        FP1_T_array[:,i] = T_i
        FP1_theta_opt_array[i] = theta_opt_i

        # Blended 2
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d, F_d_d_i, previous_theta=B2_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.BLENDED_2)
        B2_theta_opt_prev = theta_opt_i
        B2_T_array[:,i] = T_i
        B2_theta_opt_array[i] = theta_opt_i

        # ZThrust, Blended 2
        theta_z_i, T_z_i = find_pitch_thrust_ZThrust(v_d_d, F_d_d_i, model=AERO_TYPE.BLENDED_2)
        T_z_array[:,i] = T_z_i
        theta_z_array[i] = theta_z_i

    fig, ax = plot_general_thrust_pitch(
        F_x,
        [SA_T_array, FP1_T_array, B2_T_array, T_z_array],
        [SA_theta_opt_array, FP1_theta_opt_array, B2_theta_opt_array, theta_z_array],
        r"$F_x$ (N)",
        [r"SA, Opt", r"FP1, Opt", r"B2, Opt", r"B2, $\angle \boldsymbol{F}_d^d$"],
    )

    if save:
        fig.savefig("sim_data/constant_V_gamma_changing_F" + file_type, dpi=600)

def plot_constant_F_gamma_increasing_V(save=False, nsamples=200):
    # same force, increasing velocity, same gamma
    F_d_d = np.array([2., 0., -VTOL.mass*VTOL.gravity])
    Va = np.arange(.1, 50, .1)
    gamma = 0.
    v_d_d = Va * np.array([np.cos(gamma), 0., -np.sin(gamma)]).reshape(-1,1)

    nsteps = len(Va)

    SA_T_array = np.zeros((2,nsteps))
    SA_theta_opt_array = np.zeros(nsteps)
    FP1_T_array = np.zeros((2,nsteps))
    FP1_theta_opt_array = np.zeros(nsteps)
    B2_T_array = np.zeros((2,nsteps))
    B2_theta_opt_array = np.zeros(nsteps)

    SA_theta_opt_prev = None
    FP1_theta_opt_prev = None
    B2_theta_opt_prev = None

    T_z_array = np.zeros((2,nsteps))
    theta_z_array = np.zeros(nsteps)

    for i in range(nsteps):
        v_d_d_i = v_d_d[:,i]

        # Small Angle
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d_i, F_d_d, previous_theta=SA_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.SMALL_ANGLE)
        SA_theta_opt_prev = theta_opt_i
        SA_T_array[:,i] = T_i
        SA_theta_opt_array[i] = theta_opt_i

        # Flat Plate 1
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d_i, F_d_d, previous_theta=FP1_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.FLAT_PLATE_1)
        FP1_theta_opt_prev = theta_opt_i
        FP1_T_array[:,i] = T_i
        FP1_theta_opt_array[i] = theta_opt_i

        # Blended 2
        theta_opt_i, T_i = find_pitch_thrust_sampled(v_d_d_i, F_d_d, previous_theta=B2_theta_opt_prev, nsamples=nsamples, model=AERO_TYPE.BLENDED_2)
        B2_theta_opt_prev = theta_opt_i
        B2_T_array[:,i] = T_i
        B2_theta_opt_array[i] = theta_opt_i

        # ZThrust, Blended 2
        theta_z_i, T_z_i = find_pitch_thrust_ZThrust(v_d_d_i, F_d_d, model=AERO_TYPE.BLENDED_2)
        T_z_array[:,i] = T_z_i
        theta_z_array[i] = theta_z_i

    fig, ax = plot_general_thrust_pitch(
        Va,
        [SA_T_array, FP1_T_array, B2_T_array, T_z_array],
        [SA_theta_opt_array, FP1_theta_opt_array, B2_theta_opt_array, theta_z_array],
        r"$V_a$ ($m/s$)",
        [r"SA, Opt", r"FP1, Opt", r"B2, Opt", r"B2, $\angle \boldsymbol{F}_d^d$"],
    )

    if save:
        fig.savefig("sim_data/constant_F_gamma_increasing_V" + file_type, dpi=600)

def plot_CLCD_model_comparison(save=False):
    alpha = np.linspace(-np.pi, np.pi, 500, endpoint=True)
    alpha_deg = np.rad2deg(alpha)

    CL_SA =  CL(alpha, AERO_TYPE.SMALL_ANGLE_CONTINUOUS)
    CL_FP1 =  CL(alpha, AERO_TYPE.FLAT_PLATE_1)
    CL_FP2 =  CL(alpha, AERO_TYPE.FLAT_PLATE_2)
    CL_B1 =  CL(alpha, AERO_TYPE.BLENDED_1)
    CL_B2 =  CL(alpha, AERO_TYPE.BLENDED_2)

    CD_SA =  CD(alpha, AERO_TYPE.SMALL_ANGLE_CONTINUOUS)
    CD_FP1 =  CD(alpha, AERO_TYPE.FLAT_PLATE_1)
    CD_FP2 =  CD(alpha, AERO_TYPE.FLAT_PLATE_2)
    CD_B1 =  CD(alpha, AERO_TYPE.BLENDED_1)
    CD_B2 =  CD(alpha, AERO_TYPE.BLENDED_2)


    if thesis_fmt:
        fig, ax = plt.subplots(2,1, figsize=(5,4.0),sharex=True)
        plt.subplots_adjust(right=.82, hspace=.05, left=.13, bottom=.13, top=.99)
    else:
        fig, ax = plt.subplots(2,1, figsize=(3.5,2.0),sharex=True)
        plt.subplots_adjust(right=.89, hspace=.05, left=.15, bottom=.185, top=.99)

    ax[0].plot(alpha_deg, CL_SA, "-", label=r"SA")
    ax[0].plot(alpha_deg, CL_FP1, "--", label=r"FP1")
    ax[0].plot(alpha_deg, CL_FP2, "-.", label=r"FP2")
    ax[0].plot(alpha_deg, CL_B1, ":", label=r"B1")
    ax[0].plot(alpha_deg, CL_B2, "-,", label=r"B2")
    ax[0].legend(loc = "upper right", bbox_to_anchor=(1.26, 1.))
    ax[0].set_zorder(5)
    ax[0].set_ylim(-1.1, 1.1)
    ax[0].set_ylabel(r"$C_L$")
    ax[0].grid(True)

    ax[1].plot(alpha_deg, CD_SA, "-", label=r"Small Angle")
    ax[1].plot(alpha_deg, CD_FP1, "--", label=r"Flat Plate 1")
    ax[1].plot(alpha_deg, CD_FP2, "-.", label=r"Flat Plate 2")
    ax[1].plot(alpha_deg, CD_B1, ":", label=r"Blended 1")
    ax[1].plot(alpha_deg, CD_B2, "-,", label=r"Blended 2")
    ax[1].set_ylim(-.1, 2.1)
    ax[1].set_ylabel(r"$C_D$")
    ax[1].grid(True)
    ax[1].set_xlabel(r"$\alpha$ (deg)")
    ax[1].set_xticks([-180, -135, -90, -45, 0, 45, 90, 135, 180])

    if save:
        fig.savefig("sim_data/CLCD_comparison" + file_type, dpi=600)

    if thesis_fmt:
        fig, ax = plt.subplots(1,1, figsize=(5,3),sharex=True)
        plt.subplots_adjust(right=.82, hspace=.05, left=.13, bottom=.15, top=.95)
    else:
        fig, ax = plt.subplots(1,1, figsize=(3.5,1.5),sharex=True)
        plt.subplots_adjust(right=.89, hspace=.05, left=.15, bottom=.25, top=.95)

    ax.plot(alpha_deg, CL_SA/CD_SA, "-", label=r"SA")
    ax.plot(alpha_deg, CL_FP1/CD_FP1, "--", label=r"FP1")
    ax.plot(alpha_deg, CL_FP2/CD_FP2, "-.", label=r"FP2")
    ax.plot(alpha_deg, CL_B1/CD_B1, ":", label=r"B1")
    ax.plot(alpha_deg, CL_B2/CD_B2, "-,", label=r"B2")
    ax.set_ylim(-50.0, 50.0)
    ax.set_xlim(-31, 31)
    ax.set_xticks([-30, -15, 0, 15, 30])
    ax.set_ylabel(r"$C_L / C_D$")
    ax.legend(loc = "upper right", bbox_to_anchor=(1.26, 1.))
    ax.set_zorder(5)
    ax.grid(True)

    ax.set_xlabel(r"$\alpha$ (deg)")

    if save:
        fig.savefig("sim_data/CL_over_CD" + file_type, dpi=600)


plot_constant_F_gamma_increasing_V(save=True)
plot_constant_F_V_changing_gamma(save=True)
plot_constant_V_gamma_changing_F(save=True)
plot_CLCD_model_comparison(save=True)
plt.show()

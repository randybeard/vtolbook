import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import msgState
from message_types.msg_sensors import msgSensors
from message_types.msg_controls import msgControls

import parameters.simulation_parameters as SIM
import parameters.convergence_parameters as VTOL
import parameters.sensor_parameters as SENSOR
from dynamics.vtol_dynamics import vtolDynamics
from tools.rotations import Quaternion2Rotation, Quaternion2Euler, Euler2Rotation, Euler2Quaternion
import matplotlib.pyplot as plt


def compute_delta_2(Forces):

    Fx = Forces.item(0)
    Fz = Forces.item(1)
    tau_x = Forces.item(2)
    tau_y = Forces.item(3)
    tau_z = Forces.item(4)

    xrear = VTOL.rear_rotor_pos.item(0)
    yrear = VTOL.rear_rotor_pos.item(1)
    xright = VTOL.right_rotor_pos.item(0)
    yright = VTOL.right_rotor_pos.item(1)
    xleft = VTOL.left_rotor_pos.item(0)
    yleft = VTOL.left_rotor_pos.item(1)

    kf1 = 3.9
    kr1 = 3.25
    kf2 = 0.055
    kr2 = 0.045

    M = np.array([[0, kf1, 0, kf1, 0],
            [-kr1, 0, -kf1, 0, -kf1],
            [0, -kf2, -yright*kf1, kf2, -yleft*kf1],
            [xrear*kr1, 0, xright*kf1, 0, xleft*kf1],
            [kr2, -yright*kf1, kf2, -yleft*kf1, -kf2]])

    M_inv = np.linalg.inv(M)

    del_p = M_inv @ Forces

    delta_sr = np.arctan2(del_p.item(2),del_p.item(1))
    if delta_sr < -np.pi/6:
        delta_sr += np.pi
    throttle_right = np.sqrt(del_p.item(1)**2 + del_p.item(2)**2)

    delta_sl = np.arctan2(del_p.item(4),del_p.item(3))
    if delta_sl < -np.pi/6:
        delta_sl += np.pi
    throttle_left = np.sqrt(del_p.item(3)**2 + del_p.item(4)**2)

    return np.array([[del_p.item(0)],[throttle_right],[throttle_left],[delta_sr],[delta_sl]])


def get_nonapplied_forces(state):

    R = Euler2Rotation(state.phi, state.theta, state.psi) # rotation from body to world frame
    f_g = R.T @ np.array([[0.], [0.], [VTOL.mass * VTOL.gravity]])
    fx = f_g.item(0)
    fy = f_g.item(1)
    fz = f_g.item(2)

    p = state.p
    q = state.q
    r = state.r
    qbar = 0.5 * VTOL.rho * state.Va**2
    ca = np.cos(state.alpha)
    sa = np.sin(state.alpha)
    if state.Va > 1:
        p_nondim = p * VTOL.b / (2 * state.Va)  # nondimensionalize p
        q_nondim = q * VTOL.c / (2 * state.Va)  # nondimensionalize q
        r_nondim = r * VTOL.b / (2 * state.Va)  # nondimensionalize r
    else:
        p_nondim = 0.0
        q_nondim = 0.0
        r_nondim = 0.0

    # compute Lift and Drag coefficients
    tmp1 = np.exp(-VTOL.M * (state.alpha - VTOL.alpha0))
    tmp2 = np.exp(VTOL.M * (state.alpha + VTOL.alpha0))
    sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
    CL = (1 - sigma) * (VTOL.C_L_0 + VTOL.C_L_alpha * state.alpha) \
         + sigma * 2 * np.sign(state.alpha) * sa**2 * ca
    CD = VTOL.C_D_p + ((VTOL.C_L_0 + VTOL.C_L_alpha * state.alpha)**2)/(np.pi * VTOL.e * VTOL.AR)
    # compute Lift and Drag Forces
    F_lift = qbar * VTOL.S_wing * (
            CL
            + VTOL.C_L_q * q_nondim
    )
    F_drag = qbar * VTOL.S_wing * (
            CD
            + VTOL.C_D_q * q_nondim
    )
    # compute longitudinal forces in body frame
    fx += -ca * F_drag + sa * F_lift
    fz += -sa * F_drag - ca * F_lift
    # compute lateral forces in body frame
    fy += qbar * VTOL.S_wing * (
            VTOL.C_Y_0
            + VTOL.C_Y_beta * state.beta
            + VTOL.C_Y_p * p_nondim
            + VTOL.C_Y_r * r_nondim
    )
    # compute logitudinal torque in body frame
    My = qbar * VTOL.S_wing * VTOL.c * (
            VTOL.C_m_0
            + VTOL.C_m_alpha * state.alpha
            + VTOL.C_m_q * q_nondim
    )
    # compute lateral torques in body frame
    Mx  = qbar * VTOL.S_wing * VTOL.b * (
            VTOL.C_ell_0
            + VTOL.C_ell_beta * state.beta
            + VTOL.C_ell_p * p_nondim
            + VTOL.C_ell_r * r_nondim
    )
    Mz = qbar * VTOL.S_wing * VTOL.b * (
            VTOL.C_n_0 + VTOL.C_n_beta * state.beta
            + VTOL.C_n_p * p_nondim
            + VTOL.C_n_r * r_nondim
    )

    return np.array([[fx],[fz],[Mx],[My],[Mz]])


def saturate(inp, lower_limit, upper_limit):
    out = np.zeros((inp.size,1))
    for i in range(inp.size):
        if inp[i] > upper_limit:
            out[i] = upper_limit
        elif inp[i] < lower_limit:
            out[i] = lower_limit
        else:
            out[i] = inp.item(i)
    return out


def get_delta(F_d, state):

    C_delta = np.array([[0.0, 0.0],
                        [0.0, 0.0],
                        [0.0, VTOL.C_ell_delta_a],
                        [VTOL.C_m_delta_e, 0.0],
                        [0.0, VTOL.C_n_delta_a]])
    F_del_num = np.linalg.inv(C_delta.T @ C_delta) @ C_delta.T

    F_delta = 0.5*VTOL.rho*state.Va**2*VTOL.S_wing*C_delta

    F_0 = get_nonapplied_forces(state)

    if state.Va > 3.0:
        delta_1_unsat = F_del_num/(0.5*VTOL.rho*state.Va**2*VTOL.S_wing) @ (F_d - F_0)
    else:
        delta_1_unsat = np.zeros((2,1))

    delta_1 = saturate(delta_1_unsat, np.radians(-45.0), np.radians(45.0))

    delta_2 = compute_delta_2(F_d - F_0 - F_delta @ delta_1)

    delta_2[0:3] = saturate(delta_2[0:3], 0.0, 1.0)
    delta_2[3:] = saturate(delta_2[3:], np.radians(-15.0), np.radians(115.0))

    delta_1 = np.array([[0.5, -0.5], [0.5, 0.5]]) @ delta_1

    delta = msgControls()
    delta.elevon_right = delta_1.item(0)
    delta.elevon_left = delta_1.item(1)
    delta.throttle_rear = delta_2.item(0)
    delta.throttle_right = delta_2.item(1)
    delta.throttle_left = delta_2.item(2)
    delta.servo_right = delta_2.item(3)
    delta.servo_left = delta_2.item(4)

    return delta


def reset_dynamics(Va, theta):
    phi = np.radians(0.0)
    psi = np.radians(0.0)
    e_init = Euler2Quaternion(phi,theta,psi)
    vtol = vtolDynamics()
    vtol._state = np.array([[0.0],
                            [0.0],
                            [-100],
                            [Va],
                            [0.0],
                            [Va/15.0],
                            [e_init.item(0)],
                            [e_init.item(1)],
                            [e_init.item(2)],
                            [e_init.item(3)],
                            [0.0],
                            [0.0],
                            [0.0],
                            [np.radians(0.0)],
                            [np.radians(0.0)]])
    vtol._update_velocity_data()
    vtol._update_true_state()

    return vtol


def main():

    Va = 15.0
    theta = 20.0

    vtol = reset_dynamics(Va, theta)

    force_sweep = np.arange(-5.0, 5.0, 0.005)
    moment_sweep = force_sweep/5.0

    f_a = np.zeros((6, force_sweep.size))

    for i in range(5):
        for j in range(force_sweep.size):
            F_d = np.zeros((5,1))
            F_d[i] = force_sweep[j] if i < 2 else moment_sweep[j]
            delta = get_delta(F_d, vtol.true_state)
            vtol._state[13:] = np.array([[delta.servo_right], [delta.servo_left]])
            f_a[i,j] = vtol._forces_moments(delta)[i if i == 0 else i+1]
            vtol = reset_dynamics(Va, theta)

    fig, axs = plt.subplots(3, 2, constrained_layout=True)
    fig.suptitle('Va: %f, Theta: %f' % (Va, theta))

    line1 = axs[0,0].plot(force_sweep, force_sweep, 'r--', label='Commanded')
    line2 = axs[0,0].plot(force_sweep, f_a[0,:], 'k', label='Actual')
    axs[0,0].legend()
    axs[0,0].set_title('Fx')

    axs[0,1].plot(force_sweep, force_sweep, 'r--', force_sweep, f_a[1,:], 'k')
    axs[0,1].set_title('Fz')

    axs[1,0].plot(moment_sweep, moment_sweep, 'r--', moment_sweep, f_a[2,:], 'k')
    axs[1,0].set_title('Mx')

    axs[1,1].plot(moment_sweep, moment_sweep, 'r--', moment_sweep, f_a[3,:], 'k')
    axs[1,1].set_title('My')

    axs[2,0].plot(moment_sweep, moment_sweep, 'r--', moment_sweep, f_a[4,:], 'k')
    axs[2,0].set_title('Mz')

    plt.show()


    F_d = np.array([[0.0],[2.0],[0.4],[0.0],[0.0]])

    delta = get_delta(F_d, vtol.true_state)

    print(delta)

    vtol._state[13:] = np.array([[delta.servo_right], [delta.servo_left]])
    F_actual = vtol._forces_moments(delta)
    print('Produced Force')
    print(F_actual)


if __name__=='__main__':
    main()

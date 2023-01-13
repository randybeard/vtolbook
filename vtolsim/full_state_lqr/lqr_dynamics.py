"""
lqr_dynamics.py
    this implements a simplified version of the system dynamics using the partial state
    that is controlled by the lqr.
    x = [p, v, q].T
    u = [ax, az, omega].T

"""

import sys
sys.path.append('..')
import numpy as np

from tools.quaternions import q_circleTimes, q_conj, q_rotate, q_boxPlus, q_boxMinus, state_boxMinus, state_boxPlus
from tools.rotations import hat
from message_types.msg_controls import msgControls
import parameters.convergence_parameters as VTOL
import parameters.low_level_parameters as LP
from parameters.lqr_parameters import epsilon

g = VTOL.gravity

def es_jacobians(x_tilde, x_des, u_tilde, u_des, df_traj):
    A = df_dx(x_tilde, x_des, u_tilde, u_des, df_traj)
    B = df_du(x_tilde, x_des, u_tilde, u_des, df_traj)
    return A, B
    

def df_dx(x_tilde, x_des, u_tilde, u_des, df_traj):
    """
    Compute df_tilde/dx_tilde using newton's method
    """
    n = x_tilde.size
    A = np.zeros((n, n))
    f_eps = f_tilde(x_tilde, x_des, u_tilde, u_des, df_traj)
    for i in range(0, n):
        x_eps_p = np.copy(x_tilde)

        x_eps_p[i][0] += epsilon

        f_eps_p = f_tilde(x_eps_p, x_des, u_tilde, u_des, df_traj)

        df = (f_eps_p - f_eps)/(epsilon)
        A[:,i] = df[:,0]

    return A

def df_dx_central(x_tilde, x_des, u_tilde, u_des, df_traj):
    """
    Compute df_tilde/dx_tilde using central differencing
    """
    n = x_tilde.size
    A = np.zeros((n, n))
    for i in range(0, n):
        x_eps_p = np.copy(x_tilde)
        x_eps_m = np.copy(x_tilde)

        x_eps_p[i][0] += epsilon
        x_eps_m[i][0] -= epsilon

        f_eps_p = f_tilde(x_eps_p, x_des, u_tilde, u_des, df_traj)
        f_eps_m = f_tilde(x_eps_m, x_des, u_tilde, u_des, df_traj)

        df = (f_eps_p - f_eps_m)/(2*epsilon)
        A[:,i] = df[:,0]

    return A

def df_du(x_tilde, x_des, u_tilde, u_des, df_traj):
    """
    Compute df_tilde/du_tilde using newton's method
    """
    n = x_tilde.size
    m = u_tilde.size
    B = np.zeros((n, m))

    f_eps = f_tilde(x_tilde, x_des, u_tilde, u_des, df_traj)
    for i in range(0, m):
        u_eps_p = np.copy(u_tilde)

        u_eps_p[i][0] += epsilon

        f_eps_p = f_tilde(x_tilde, x_des, u_eps_p, u_des, df_traj)

        df = (f_eps_p - f_eps)/(epsilon)
        B[:,i] = df[:,0]

    return B

def df_du_central(x_tilde, x_des, u_tilde, u_des, df_traj):
    """
    Compute df_tilde/du_tilde using central differencing
    """
    n = x_tilde.size
    m = u_tilde.size
    B = np.zeros((n, m))

    for i in range(0, m):
        u_eps_p = np.copy(u_tilde)
        u_eps_m = np.copy(u_tilde)

        u_eps_p[i][0] += epsilon
        u_eps_m[i][0] -= epsilon

        f_eps_p = f_tilde(x_tilde, x_des, u_eps_p, u_des, df_traj)
        f_eps_m = f_tilde(x_tilde, x_des, u_eps_m, u_des, df_traj)

        df = (f_eps_p - f_eps_m)/(2*epsilon)
        B[:,i] = df[:,0]

    return B

def f_tilde(x_tilde, x_des, u_tilde, u_des, df_traj):
    """
    Compute f_tilde as done in "LQR Control of a Fixed-Wing Aircraft Using Vector Fields"
    """
    x = state_boxPlus(x_des, -x_tilde)
    u = u_des - u_tilde

    # no need to use [+] here since qdot is a 4-vector
    x_dot = f(x,u)
    x_p = x + x_dot*epsilon
    x_m = x - x_dot*epsilon

    x_des_p = x_des
    x_des_m = x_des
    
    x_tilde_p = state_boxMinus(x_des_p, x_p)
    x_tilde_m = state_boxMinus(x_des_m, x_m)

    f_tilde = (x_tilde_p - x_tilde_m)/(2*epsilon)

    return f_tilde


def ctrl_boxPlus(u1, u2):
    return u1 + u2

def ctrl_boxMinus(u1, u2):
    return u1 - u2

def f(x, u):
    """
    Simplified dynamics, assume that rates (omega) are acheived instantaneously, and airspeed=groundspeed
    """
    f_mag = np.linalg.norm(u[0:2])
    if(f_mag > .9):
        u[0:2] = .9*u[0:2] / f_mag

    if(u.item(0) < 0.0):
        u[0] = 0.0
    if(u.item(1) < 0.0):
        u[1] = 0.0

    pos       = x[0:3]
    v         = x[3:6]
    q         = x[6:10] # rotation from body to inertial
    
    omega     = u[2:5]

    omega_aug = np.array([[0, omega.item(0), omega.item(1), omega.item(2)]]).T
    e3        = np.array([[0, 0, 1]]).T

    p_dot     = q_rotate(v, q) # rotate v from body to inertial
    
    # get applied forces
    F         = forces(x, u) 
    G         = g*q_rotate(e3, q_conj(q))

    v_dot     = hat(omega) @ v + G + F/VTOL.mass

    q_dot     = 0.5*q_circleTimes(q, omega_aug) 

    x_dot = np.concatenate([p_dot, v_dot, q_dot], 0)

    return x_dot

def forces(x, u):
    omega = u[2:5]
    F     = u[0:2]
    delta = mixer(F)

    Va = x[3:6]
    # Assume airspeed = ground speed
    # Assume omega acheived instantly

    F = applied_forces(Va, delta, omega)
    return F


def mixer(F):
    """
    Given Fx and Fz convert to delta (motor commands)
    """

    #                 t_re, t_ri*c(dr), t_ri*s(dr), t_l*c(dl), t_l*s(dl)
    # mixer = np.array([[   0.0,   1.0,   0.0,   1.0,   0.0],  #Fx
                      # [   1.0,   0.0,   1.0,   0.0,   1.0]]) #Fz
    mixer = LP.mixer[:2,:5] # reduce mixer for only Fx, Fz and motor/servo outputs

    zeta = (F.T @ mixer).T

    servo_right = np.arctan2(zeta.item(2), zeta.item(1))
    servo_left = np.arctan2(zeta.item(4), zeta.item(3))
    throttle_right = np.sqrt(zeta.item(1)**2 + zeta.item(2)**2)
    throttle_left = np.sqrt(zeta.item(3)**2 + zeta.item(4)**2)
    throttle_rear = zeta.item(0)
    
    delta = np.array([[throttle_rear, throttle_right, throttle_left, servo_right, servo_left]]).T

    # saturate 

    limits = LP.limits[:, :5] # limits only for throttle/servo
    for i in range(0, delta.shape[0]):
        delta[i] = sat(delta.item(i), limits.item(0,i), limits.item(1,i))
    
    output = msgControls()
    output.throttle_rear = delta.item(0)
    output.throttle_right = delta.item(1)
    output.throttle_left = delta.item(2)
    output.servo_right = delta.item(3)
    output.servo_left = delta.item(4)
    return output


def sat(val, low, high):
    if val < low:
        val = low
    elif val > high:
        val = high
    return val
    

def applied_forces(Va, delta, omega):
    """
    return the forces on the UAV based on the state, wind, and control surfaces
    :param: Va the airspeed vector
    :param: delta the actuator commands (msgControls)
    :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
    """
    
    elevator = delta.elevon_right + delta.elevon_left
    aileron = -delta.elevon_right + delta.elevon_left
    rudder = 0.0  # the convergence doesn't have a rudder

    p = omega.item(0)
    q = omega.item(1)
    r = omega.item(2)

    norm_Va = np.linalg.norm(Va)
    alpha = np.arctan2(Va.item(2), Va.item(0))
    beta = np.arctan2(Va.item(1), norm_Va)

    # intermediate variables
    qbar = 0.5 * VTOL.rho * norm_Va**2
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    if norm_Va > 1:
        p_nondim = p * VTOL.b / (2 * norm_Va)  # nondimensionalize p
        q_nondim = q * VTOL.c / (2 * norm_Va)  # nondimensionalize q
        r_nondim = r * VTOL.b / (2 * norm_Va)  # nondimensionalize r
    else:
        p_nondim = 0.0
        q_nondim = 0.0
        r_nondim = 0.0

    # compute Lift and Drag coefficients
    tmp1 = np.exp(-VTOL.M * (alpha - VTOL.alpha0))
    tmp2 = np.exp(VTOL.M * (alpha + VTOL.alpha0))
    sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
    CL = (1 - sigma) * (VTOL.C_L_0 + VTOL.C_L_alpha * alpha) \
         + sigma * 2 * np.sign(alpha) * sa**2 * ca
    CD = VTOL.C_D_p + ((VTOL.C_L_0 + VTOL.C_L_alpha * alpha)**2)/(np.pi * VTOL.e * VTOL.AR)
    # compute Lift and Drag Forces
    F_lift = qbar * VTOL.S_wing * (
            CL
            + VTOL.C_L_q * q_nondim
            + VTOL.C_L_delta_e * elevator
    )
    F_drag = qbar * VTOL.S_wing * (
            CD
            + VTOL.C_D_q * q_nondim
            + VTOL.C_D_delta_e * elevator
    )
    # compute longitudinal forces in body frame
    fx = - ca * F_drag + sa * F_lift
    fz = - sa * F_drag - ca * F_lift
    # compute lateral forces in body frame
    fy = qbar * VTOL.S_wing * (
            VTOL.C_Y_0
            + VTOL.C_Y_beta * beta
            + VTOL.C_Y_p * p_nondim
            + VTOL.C_Y_r * r_nondim
            + VTOL.C_Y_delta_a * aileron
            + VTOL.C_Y_delta_r * rudder
    )

    # force and torque from rear motor
    Va_rear = np.array([[0.0], [0.0], [-1.0]]).T @ Va 
    Thrust, _ = _motor_thrust_torque(Va_rear, delta.throttle_rear, True)
    Force_rear = Thrust * np.array([[0.0],
                                    [0.0],
                                    [-1.0]])
    fx += Force_rear.item(0)
    fy += Force_rear.item(1)
    fz += Force_rear.item(2)

    # thrust and torque from the right motor
    Va_right = np.array([[np.cos(delta.servo_right)],
                        [0.0],
                        [-np.sin(delta.servo_right)]]).T @ Va
    Thrust, _ = _motor_thrust_torque(Va_right, delta.throttle_right, False)
    Force_right = Thrust * np.array([[np.cos(delta.servo_right)],
                                     [0.0],
                                     [-np.sin(delta.servo_right)]])
    fx += Force_right.item(0)
    fy += Force_right.item(1)
    fz += Force_right.item(2)

    # thrust and torque from the left motor
    Va_left = np.array([[np.cos(delta.servo_left)],
                        [0.0],
                        [-np.sin(delta.servo_left)]]).T @ Va
    Thrust, _ = _motor_thrust_torque(Va_left, delta.throttle_left, False)
    Force_left = Thrust * np.array([[np.cos(delta.servo_left)],
                                    [0.0],
                                    [-np.sin(delta.servo_left)]])
    fx += Force_left.item(0)
    fy += Force_left.item(1)
    fz += Force_left.item(2)

    return np.array([[fx, fy, fz]]).T

def _motor_thrust_torque(Va, delta_t, is_rear):
    # compute thrust and torque due to propeller  (See addendum by McLain)
    # grab motor/prop params
    if is_rear:
        C_Q0 = VTOL.C_Q0_rear
        C_Q1 = VTOL.C_Q1_rear
        C_T0 = VTOL.C_T0_rear
        C_Q2 = VTOL.C_Q2_rear
        C_T1 = VTOL.C_T1_rear
        C_T2 = VTOL.C_T2_rear
        D_prop = VTOL.D_prop_rear
        KQ = VTOL.KQ_rear
        R_motor = VTOL.R_motor_rear
        i0 = VTOL.i0_rear
    else:
        C_Q0 = VTOL.C_Q0_front
        C_Q1 = VTOL.C_Q1_front
        C_Q2 = VTOL.C_Q2_front
        C_T0 = VTOL.C_T0_front
        C_T1 = VTOL.C_T1_front
        C_T2 = VTOL.C_T2_front
        D_prop = VTOL.D_prop_front
        KQ = VTOL.KQ_front
        R_motor = VTOL.R_motor_front
        i0 = VTOL.i0_front
    # map delta_t throttle command(0 to 1) into motor input voltage
    V_in = VTOL.V_max * delta_t

    # Quadratic formula to solve for motor speed
    D_pow_5 = np.power(D_prop, 5)
    D_pow_4 = np.power(D_prop, 4)
    a = C_Q0 * VTOL.rho * D_pow_5 \
        / ((2.*np.pi)**2)
    b = (C_Q1 * VTOL.rho * D_pow_4
         / (2.*np.pi)) * Va + KQ**2/R_motor
    c = C_Q2 * VTOL.rho * np.power(D_prop, 3) \
        * Va**2 - (KQ / R_motor) * V_in + KQ * i0

    # Consider only positive root
    Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)

    # compute advance ratio
    J_op = 2 * np.pi * Va / (Omega_op * D_prop)

    # compute non-dimensionalized coefficients of thrust and torque
    J_op_pow_2 = J_op**2
    C_T = C_T2 * J_op_pow_2 + C_T1 * J_op + C_T0
    C_Q = C_Q2 * J_op_pow_2 + C_Q1 * J_op + C_Q0

    # add thrust and torque due to propeller
    n = Omega_op / (2 * np.pi)
    n_pow_2 = n**2
    T_p = VTOL.rho * n_pow_2 * D_pow_4 * C_T
    Q_p = VTOL.rho * n_pow_2 * D_pow_5 * C_Q

    return T_p, Q_p


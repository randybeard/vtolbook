"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        2/4/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from tools.rotations import Euler2Quaternion, Quaternion2Euler
from tools.transfer_function import transferFunction
import parameters.aerosonde_parameters as MAV
from parameters.simulation_parameters import ts_simulation as Ts

def compute_tf_model(mav, trim_state, trim_input):
    # trim values
    mav._state = trim_state
    mav._update_velocity_data()
    Va_trim = mav._Va
    alpha_trim = mav._alpha
    phi, theta_trim, psi = Quaternion2Euler(trim_state[6:10])

    # define transfer function constants
    a_phi1 = -0.5 * MAV.rho * Va_trim**2 * MAV.S_wing * MAV.b * MAV.C_p_p * MAV.b / 2.0 / Va_trim
    a_phi2 = 0.5 * MAV.rho * Va_trim**2 * MAV.S_wing * MAV.b * MAV.C_p_delta_a
    a_theta1 = -MAV.rho * Va_trim**2 * MAV.c * MAV.S_wing / 2.0 / MAV.Jy * MAV.C_m_q * MAV.c / 2.0 / Va_trim
    a_theta2 = -MAV.rho * Va_trim**2 * MAV.c * MAV.S_wing / 2.0 / MAV.Jy * MAV.C_m_alpha
    a_theta3 = MAV.rho * Va_trim**2 * MAV.c * MAV.S_wing / 2.0 / MAV.Jy * MAV.C_m_delta_e
    a_beta1 = -(MAV.rho * Va_trim * MAV.S_wing) / 2.0 / MAV.mass * MAV.C_Y_beta
    a_beta2 = (MAV.rho * Va_trim**2 * MAV.S_wing) / 2.0 / MAV.mass * MAV.C_Y_delta_r

    #feedforwardgain = a_theta2 / a_theta3

    # Compute transfer function coefficients using new propulsion model
    delta_e_trim = trim_input.item(0)
    delta_t_trim = trim_input.item(3)
    a_V1 = MAV.rho * Va_trim * MAV.S_wing / MAV.mass * (
            MAV.C_D_0
            + MAV.C_D_alpha * alpha_trim
            + MAV.C_D_delta_e * delta_e_trim
            ) - dT_dVa(mav, Va_trim, delta_t_trim)/MAV.mass
    a_V2 = dT_ddelta_t(mav, Va_trim, delta_t_trim) / MAV.mass
    a_V3 = MAV.gravity * np.cos(theta_trim - alpha_trim)

# write transfer function gains to file
    file = open('transfer_function_coef.py', 'w')
    file.write('import numpy as np\n')
    file.write('x_trim = np.array([[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]]).T\n' %
               (trim_state.item(0), trim_state.item(1), trim_state.item(2), trim_state.item(3),
                trim_state.item(4), trim_state.item(5), trim_state.item(6), trim_state.item(7),
                trim_state.item(8), trim_state.item(9), trim_state.item(10), trim_state.item(11),
                trim_state.item(12)))
    file.write('u_trim = np.array([[%f, %f, %f, %f]]).T\n' %
               (trim_input.item(0), trim_input.item(1), trim_input.item(2), trim_input.item(3)))
    file.write('Va_trim = %f\n' % Va_trim)
    file.write('alpha_trim = %f\n' % alpha_trim)
    file.write('theta_trim = %f\n' % theta_trim)
    file.write('a_phi1 = %f\n' % a_phi1)
    file.write('a_phi2 = %f\n' % a_phi2)
    file.write('a_theta1 = %f\n' % a_theta1)
    file.write('a_theta2 = %f\n' % a_theta2)
    file.write('a_theta3 = %f\n' % a_theta3)
    file.write('a_beta1 = %f\n' % a_beta1)
    file.write('a_beta2 = %f\n' % a_beta2)
    file.write('a_V1 = %f\n' % a_V1)
    file.write('a_V2 = %f\n' % a_V2)
    file.write('a_V3 = %f\n' % a_V3)
    file.write('Ts = %f\n' % Ts)
    file.close()

# define transfer functions
    T_phi_delta_a = transferFunction(np.array([[a_phi2]]), np.array([[1, a_phi1, 0]]), Ts)
    T_chi_phi = transferFunction(np.array([[MAV.gravity / Va_trim]]), np.array([[1, 0]]), Ts)
    T_beta_delta_r = transferFunction(np.array([[a_beta2]]), np.array([[1, a_beta1]]), Ts)
    T_theta_delta_e = transferFunction(np.array([[a_theta3]]), np.array([[1, a_theta1, a_theta2]]), Ts)
    T_h_theta = transferFunction(np.array([[Va_trim]]), np.array([[1, 0]]), Ts)
    T_h_Va = transferFunction(np.array([[theta_trim]]), np.array([[1, 0]]), Ts)
    T_Va_delta_t = transferFunction(np.array([[a_V2]]), np.array([[1, a_V1]]), Ts)
    T_Va_theta = transferFunction(np.array([[-a_V3]]), np.array([[1, a_V1]]), Ts)

    return T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_beta_delta_r

def compute_ss_model(mav, trim_state, trim_input):
    x_euler = euler_state(trim_state)
    A = df_dx(mav, x_euler, trim_input)
    B = df_du(mav, x_euler, trim_input)
    # extract longitudinal states (u, w, q, theta, pd)
    A_lon = A[np.ix_([3, 5, 10, 7, 2],[3, 5, 10, 7, 2])]
    B_lon = B[np.ix_([3, 5, 10, 7, 2],[0, 3])]
    # change pd to h
    for i in range(0, 5):
        A_lon[i, 4] = -A_lon[i, 4]
        A_lon[4, i] = -A_lon[4, i]
    for i in range(0, 2):
        B_lon[4, i] = -B_lon[4, i]
    # extract lateral states (v, p, r, phi, psi)
    A_lat = A[np.ix_([4, 9, 11, 6, 8],[4, 9, 11, 6, 8])]
    B_lat = B[np.ix_([4, 9, 11, 6, 8],[1, 2])]
    return A_lon, B_lon, A_lat, B_lat

def euler_state(x_quat):
    # convert state x with attitude represented by quaternion
    # to x_euler with attitude represented by Euler angles
    x_euler = np.zeros((12,1))
    x_euler[0:6] = np.copy(x_quat[0:6])  # copy position, velocity
    phi, theta, psi = Quaternion2Euler(x_quat[6:10])
    x_euler[6] = phi
    x_euler[7] = theta
    x_euler[8] = psi
    x_euler[9:12] = np.copy(x_quat[10:13]) # copy angular rate
    return x_euler

def quaternion_state(x_euler):
    # convert state x_euler with attitude represented by Euler angles
    # to x_quat with attitude represented by quaternions
    x_quat = np.zeros((13,1))
    x_quat[0:6] = np.copy(x_euler[0:6])  # copy position, velocity
    phi = x_euler.item(6)
    theta = x_euler.item(7)
    psi = x_euler.item(8)
    quat = Euler2Quaternion(phi, theta, psi)
    x_quat[6:10] = quat
    x_quat[10:13] = np.copy(x_euler[9:12]) # copy angular rate
    return x_quat

def f_euler(mav, x_euler, input):
    # return 12x1 dynamics (as if state were Euler state)
    # compute f at euler_state
    x_quat = quaternion_state(x_euler)
    mav._state = x_quat
    mav._update_velocity_data()
    f = mav._derivatives(x_quat, mav._forces_moments(input))
    # f_euler will be f, except for the attitude states
    f_euler_ = euler_state(f)

    # need to correct attitude states by multiplying f by
    # partial of Quaternion2Euler(quat) with respect to quat
    # compute partial Quaternion2Euler(quat) with respect to quat
    eps = 0.001  # epsilon used to approximate derivatives
    e = x_quat[6:10]
    phi = x_euler.item(6)
    theta = x_euler.item(7)
    psi = x_euler.item(8)
    dTheta_dquat = np.zeros((3,4))
    for j in range(0, 4):
        tmp = np.zeros((4, 1))
        tmp[j][0] = eps
        #e_eps = (e + tmp)
        e_eps = (e + tmp) / np.linalg.norm(e + tmp)
        phi_eps, theta_eps, psi_eps = Quaternion2Euler(e_eps)
        dTheta_dquat[0][j] = (phi_eps - phi) / eps
        dTheta_dquat[1][j] = (theta_eps - theta) / eps
        dTheta_dquat[2][j] = (psi_eps - psi) / eps
    # Multiply f by
    # partial of Quaternion2Euler(quat) with respect to quat
    f_euler_[6:9] = np.copy(dTheta_dquat @ f[6:10])
    return f_euler_

def df_dx(mav, x_euler, input):
    # take partial of f_euler with respect to x_euler
    eps = 0.01  # deviation
    A = np.zeros((12, 12))  # Jacobian of f wrt x
    f = f_euler(mav, x_euler, input)
    for i in range(0, 12):
        x_eps = np.copy(x_euler)
        x_eps[i][0] += eps
        f_eps = f_euler(mav, x_eps, input)
        df = (f_eps - f) / eps
        A[:,i] = df[:,0]
    return A

def df_du(mav, x_euler, delta):
    # take partial of f_euler with respect to delta
    eps = 0.01  # deviation
    B = np.zeros((12, 4))  # Jacobian of f wrt u
    f = f_euler(mav, x_euler, delta)
    for i in range(0, 4):
        delta_eps = np.copy(delta)
        delta_eps[i, 0] += eps
        f_eps = f_euler(mav, x_euler, delta_eps)
        df = (f_eps - f) / eps
        B[:,i] = df[:,0]
    return B

def dT_dVa(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to Va
    eps = 0.01
    T_eps, Q_eps = mav._motor_thrust_torque(Va+eps, delta_t)
    T, Q = mav._motor_thrust_torque(Va, delta_t)
    return (T_eps - T) / eps

def dT_ddelta_t(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to delta_t
    eps = 0.01
    T_eps, Q_eps = mav._motor_thrust_torque(Va, delta_t+eps)
    T, Q = mav._motor_thrust_torque(Va, delta_t)
    return (T_eps - T) / eps
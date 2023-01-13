"""
    lqr controller for vtolsim
"""
import sys
import numpy as np
sys.path.append('..')
import scipy
from tools.wrap import wrap
from message_types.msg_state import msgState
from message_types.msg_controls import msgControls
from tools.msg_convert import *
from full_state_lqr.lqr_dynamics import es_jacobians
from tools.quaternions import state_boxMinus, state_boxPlus
from parameters.lqr_parameters import Q, R

def find_K(A, B):
    # set up Hamiltonian matrix
    R_inv = np.linalg.inv(R)
    H = np.block([[A, -B @ R_inv @ B.T], [-Q, -A.T]])
    # find schur decomposition
    try:
        T, U, sdim = scipy.linalg.schur(H, sort='lhp')
    except np.linalg.LinAlgError as err:
        print("----- Error on Schur Decomp -----")
        print(err)
        print("\n\nA = \n", A)
        print("\n\nB = \n", B)
        print("\n\nH = \n", H)
        raise

    n = int(np.size(U, 0)/2)
    U_11 = U[0:n, 0:n]
    U_21 = U[n:, 0:n]
    # find P
    P = np.linalg.inv(U_11.T) @ U_21.T
    K = R_inv @ B.T @ P
    # print("K = ", K)
    return K

def update(x, x_des, u, u_des, df_traj):
    """
    Control state = (p, v, q)
    Control Input = (ax, az, omega)
    """
    # find error state
    x_tilde = state_boxMinus(x_des, x)
    u_tilde = u_des - u

    # print performance measure
    # J_x = (x_tilde.T @ Q).T * x_tilde
    # J_u = (u_tilde.T @ R).T * u_tilde
    # print("J_x = ", np.sum(J_x), " = ", J_x.T)
    # print("J_u = ", np.sum(J_u), " = ", J_u.T)
    
    x_tilde[0:2, 0] = np.clip(x_tilde[0:2, 0], -.5, .5)

    # find A and B matrices
    A, B = es_jacobians(x_tilde, x_des, u_tilde, u_des, df_traj) 
    

    # find K gain matrix
    K = find_K(A, B)

    u_star = K @ x_tilde
    u = u_des + u_star

    return u





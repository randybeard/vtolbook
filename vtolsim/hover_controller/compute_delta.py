import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
import parameters.convergence_parameters as VTOL
from message_types.msg_controls import msgControls
import time
import math



def compute_forces(delta):
    """
        convert the delta commands to forces
        delta = [elevon_right, elevon_left, throttle_rear, throttle_right, throttle_left, servo_right, servo_left].T
    """
    kf1 = 3.5
    kr1 = 2.75
    kf2 = 0.055
    kr2 = 0.045

    
    elevon_right    = delta.item(0) # passthrough
    elevon_left     = delta.item(1) # passthrough
    throttle_rear   = delta.item(2)
    throttle_right  = delta.item(3)
    throttle_left   = delta.item(4)
    servo_right     = delta.item(5)
    servo_left      = delta.item(6)

    Fx = kf1*throttle_right*np.cos(servo_right) + kf1*throttle_left*np.cos(servo_left)
    Fz = -kr1*throttle_rear - kf1*throttle_right*np.sin(servo_right) - kf1*throttle_left*np.sin(servo_left)
    M_rear = kr2*throttle_rear*np.array([[0],[0],[1]]) + np.cross(VTOL.rear_rotor_pos.T, np.array([[0],[0],[-kr1*throttle_rear]]).T).T
    M_right = kf2*throttle_right*np.array([[-np.cos(servo_right)],[0],[np.sin(servo_right)]]) + np.cross(VTOL.right_rotor_pos.T, np.array([[kf1*throttle_right*np.cos(servo_right)],[0],[-kf1*throttle_right*np.sin(servo_right)]]).T).T
    M_left = kf2*throttle_left*np.array([[np.cos(servo_left)],[0],[-np.sin(servo_left)]]) + np.cross(VTOL.left_rotor_pos.T, np.array([[kf1*throttle_left*np.cos(servo_left)],[0],[-kf1*throttle_left*np.sin(servo_left)]]).T).T
    M = M_rear + M_right + M_left
    return np.concatenate((np.array([[elevon_right], [elevon_left], [Fx],[0.0],[Fz]]), M), axis=0)

def J(delta, Forces):
    return np.linalg.norm(Forces-compute_forces(delta))**2

def compute_grad(delta, Forces):
    e = 0.0001
    grad = np.zeros((5,1))
    for i in range(len(grad)):
        delta_pert = np.copy(delta)
        delta_pert[i] += e
        grad[i] = (J(np.copy(delta_pert), Forces) - J(np.copy(delta), Forces))/e
    return grad

def compute_delta_gd(Forces):
    delta0 = np.array([[0.0],[0.0],[0.0],[np.radians(0.0)],[np.radians(0.0)]])#throttle_rear, throttle_right, throttle_left, servo_right, servo_left

    alpha = 0.012
    delta = np.copy(delta0)
    cost = J(np.copy(delta), Forces)
    while cost > 1e-5:
        grad = compute_grad(delta, Forces)
        delta -= alpha*grad
        cost = J(np.copy(delta), Forces)
        # print(cost)

    return delta

def compute_delta_sci(Forces):
    delta = np.array([[0.0],[0.0],[0.0],[0.0],[0.0]])
    res = minimize(J, delta, method='SLSQP', args = (Forces),
                   options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000})
    return np.array([res.x]).T


def compute_delta(u_input):

    elev_r = u_input.item(0) #passthrough
    elev_l = u_input.item(1) #passthrough
    Fx = u_input.item(2)
    Fy = u_input.item(3)
    Fz = u_input.item(4)
    tau_x = u_input.item(5)
    tau_y = u_input.item(6)
    tau_z = u_input.item(7)

    Forces = np.array([[Fx, Fz, tau_x, tau_y, tau_z]]).T

    xrear = VTOL.rear_rotor_pos.item(0)
    yrear = VTOL.rear_rotor_pos.item(1)
    xright = VTOL.right_rotor_pos.item(0)
    yright = VTOL.right_rotor_pos.item(1)
    xleft = VTOL.left_rotor_pos.item(0)
    yleft = VTOL.left_rotor_pos.item(1)

    kf1 = 3.5 + 0.4
    kr1 = 2.75 + 0.5
    kf2 = 0.055
    kr2 = 0.045

    M = np.array([[0, kf1, 0, kf1, 0],
            [-kr1, 0, -kf1, 0, -kf1],
            [0, -kf2, -yright*kf1, kf2, -yleft*kf1],
            [xrear*kr1, 0, xright*kf1, 0, xleft*kf1],
            [kr2, -yright*kf1, kf2, -yleft*kf1, -kf2]])

    M_inv = np.linalg.inv(M)

    del_p = M_inv @ Forces

    delta_sr = math.atan2(del_p.item(2),del_p.item(1))
    if delta_sr < -math.pi/6:
        delta_sr += math.pi
    throttle_right = math.sqrt(del_p.item(1)**2 + del_p.item(2)**2)

    delta_sl = math.atan2(del_p.item(4),del_p.item(3))
    if delta_sl < -math.pi/6:
        delta_sl += math.pi
    throttle_left = math.sqrt(del_p.item(3)**2 + del_p.item(4)**2)

    return np.array([[elev_r], [elev_l], [del_p.item(0)],[throttle_right],[throttle_left],[delta_sr],[delta_sl]])


def main():
    # forces = np.array([[5.0],[0.0],[0.0],[0.0],[0.0]])

    ans = np.array([[0.1],[0.8],[0.75],[np.radians(110.0)],[np.radians(-15.0)]])
    forces = compute_forces(ans)
    start = time.time()
    delta = compute_delta(forces)
    print(time.time()-start)
    print("Got em!")
    print(delta)


if __name__=='__main__':
    main()

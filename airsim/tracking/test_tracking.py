import time
import numpy as np
import pathlib
import sys
import cv2
import keyboard

# this line gets the parent folder of the parent folder of the current file
# i.e. vtolbook/airsim/. If your sim file is in the airsim folder, you don't need this
directory = pathlib.Path(__file__).parent.parent
# this is so we can have the base folder of the project (vtolbook/airsim) in the path
sys.path.append(str(directory))

import parameters.quadrotor_parameters as QUAD
import parameters.simulation_parameters as SIM
from viz.quad_viewer import QuadViewer
from dynamics.quad_dynamics import QuadDynamics
# from controllers.jakes_controller import Autopilot
from controllers.velocity_controller_2 import Autopilot
from trajectory_generators.circular_trajectory import TrajectoryGenerator
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot
from message_types.msg_state import MsgState
from ball_detector import BallDetector
from tools.rotations import rotation_to_euler, rot_x, rot_y

airsim_viz = QuadViewer()
quadrotor = QuadDynamics(SIM.ts_simulation, QUAD)
autopilot = Autopilot(SIM.ts_simulation, QUAD)
traj_gen = TrajectoryGenerator(SIM.ts_simulation)
traj_msg = MsgAutopilot()
delta = MsgDelta()
ball_detector = BallDetector()
sim_time = SIM.start_time

quadrotor.true_state.pos[2] = -10
airsim_viz.update(quadrotor.true_state)
time.sleep(1)
airsim_viz.spawn_target()

    
np.set_printoptions(precision=4,suppress=True)
alpha = np.deg2rad(-45)

psi = 0.0

while sim_time < SIM.end_time:
    velocity = np.array([[0],[0],[0]])

    val = 4

    if keyboard.is_pressed('left_arrow'):
        velocity[1] = -val
    if keyboard.is_pressed('right_arrow'):
        velocity[1] = val
    if keyboard.is_pressed('up_arrow'):
        velocity[0] = val
    if keyboard.is_pressed('down_arrow'):
        velocity[0] = -val
    
    state = quadrotor.true_state
    
    img = airsim_viz.get_image("angled")
    # cv2.imshow("angled", img)
    # cv2.waitKey(1)

    px_location = ball_detector.detect(img,"angled")
    # px_location = (-px_location[1],px_location[0])
    calibrated_px = QUAD.K_inv @ np.append(px_location,1).T
    e_f_c_bar = (calibrated_px/np.linalg.norm(calibrated_px)).reshape(-1,1)

    
    phi,theta,psi = rotation_to_euler(state.rot)
    R_b_l = rot_y(theta) @ rot_x(phi)

    R_cam_to_ned = np.array([[0,   0,  1],
                             [1,   0,  0],
                             [0,   1,  0]])
    
    R_c_b =  R_cam_to_ned @ rot_x(alpha)

    e_f_l = R_b_l @ R_c_b @ e_f_c_bar

    e_f_l_bar = e_f_l / e_f_l.item(2)

    gamma_f_c = 1
    gamma_f_l = gamma_f_c * np.array([[0],[0],[1]]).T @ R_b_l @ R_c_b @ e_f_c_bar

    # e_f_l_bar /= np.linalg.norm(e_f_l_bar)
    
    
    m_t = gamma_f_l * e_f_l_bar / np.linalg.norm(gamma_f_l * e_f_l_bar)

    # m_t[2] *= -1

    # print()
    # print(m_t)

    # print()
    # print(m_t)

    
    m_d = np.array([np.cos(alpha), 0, -np.sin(alpha)]).reshape(-1,1)
    # m_d = np.array([[0],[0],[0]])
    # m_d = np.array([[-.8],[.2],[0.0025]])

    gamma_m_d = np.eye(3,3) - m_d@ m_d.T
    
    e3 = np.array([[0],[0],[1]])
    gamma_e_3 = np.eye(3,3) - e3 @ e3.T
    
    nu_1 = gamma_m_d @ m_t
    
    nu = 10 * gamma_e_3 @ gamma_m_d @ m_t

    if np.linalg.norm(gamma_e_3 @ velocity) > 0:
        heading_vector = gamma_e_3 @ velocity / np.linalg.norm(gamma_e_3 @ velocity)
        if heading_vector.item(0) > 0:
            psi = np.arccos(heading_vector.item(0))
        else:
            psi = np.arcsin(heading_vector.item(1))
    # nu[0] = 0
    # print("M_t: ",m_t)
    # print("M_d: ",m_d)
    print()
    print(nu)

    traj_msg.vel = nu
    traj_msg.heading = psi
    # traj_msg = traj_gen.update()


    estimated_state = quadrotor.true_state

    delta, commanded_states = autopilot.update(traj_msg, estimated_state)
    quadrotor.update(delta)

    
    airsim_viz.update_target(velocity,SIM.ts_simulation)
    airsim_viz.update(quadrotor.true_state)

    


 
    
    # increment the simulation
    sim_time += SIM.ts_simulation
    # time.sleep(SIM.ts_simulation)


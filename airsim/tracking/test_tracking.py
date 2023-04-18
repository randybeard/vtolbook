import time
import numpy as np
import pathlib
import sys
import cv2 


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

while sim_time < SIM.end_time:
    

    img = airsim_viz.get_image("angled")
    # cv2.imshow("angled", img)
    # cv2.waitKey(1)

    px_location = ball_detector.detect(img,"angled")
    px_location = (px_location[1],px_location[0])
    calibrated_px = QUAD.K_inv @ np.append(px_location,1).T
    m_t = (calibrated_px/np.linalg.norm(calibrated_px)).reshape(-1,1)
    # print()
    # print(m_t)

    alpha = np.deg2rad(45)
    # m_d = np.array([np.cos(alpha), 0, np.sin(alpha)]).reshape(-1,1)
    m_d = np.array([[0],[0],[0]])

    gamma_m_d = np.eye(3,3) - m_d@ m_d.T
    
    e3 = np.array([[0],[0],[1]])
    gamma_e_3 = np.eye(3,3) - e3 @ e3.T
    
    nu_1 = gamma_m_d @ m_t
    
    nu = 10 * gamma_e_3 @ gamma_m_d @ m_t
    nu[0] = 0
    # print("M_t: ",m_t)
    # print("M_d: ",m_d)
    print()
    print(nu)

    traj_msg.vel = nu
    # traj_msg = traj_gen.update()


    estimated_state = quadrotor.true_state

    delta, commanded_states = autopilot.update(traj_msg, estimated_state)
    quadrotor.update(delta)

    airsim_viz.update_target([0,-2,0],SIM.ts_simulation)
    airsim_viz.update(quadrotor.true_state)


 
    
    # increment the simulation
    sim_time += SIM.ts_simulation
    # time.sleep(SIM.ts_simulation)

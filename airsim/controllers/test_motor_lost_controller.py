import time
import numpy as np
import pathlib
import sys

# this line gets the parent folder of the parent folder of the current file
# i.e. vtolbook/airsim/. If your sim file is in the airsim folder, you don't need this
directory = pathlib.Path(__file__).parent.parent
# this is so we can have the base folder of the project (vtolbook/airsim) in the path
sys.path.append(str(directory))

from copy import deepcopy
import parameters.detalied_quadrotor_parameters as QUAD
import parameters.simulation_parameters as SIM
from viz.quad_viewer import QuadViewer
from dynamics.detailed_dynamics import QuadDynamics
from controllers.motor_lost_controller import Autopilot
from message_types.msg_autopilot import MsgAutopilot

SIM.ts_simulation = 0.001
SIMULATE = True
# SIMULATE = False
if SIMULATE:
    airsim_viz = QuadViewer()
quadrotor = QuadDynamics(SIM.ts_simulation, QUAD)
autopilot = Autopilot(SIM.ts_simulation, QUAD)
sim_time = SIM.start_time

des_pos = np.array([[0., 0., 3.]]).T
new_pos = np.array([[1., 1., 3.]]).T
time_switch = 6

des_traj = MsgAutopilot()
des_traj.pos = des_pos

while sim_time < SIM.end_time:
    if sim_time > time_switch:
        des_traj.pos = new_pos
    # get the estimated state (the estimate is ground truth here)
    estimated_state = quadrotor.true_state

    # get the commands from the controller
    delta, commanded_states = autopilot.update(des_traj, estimated_state)

    # update the dynamics and simulation
    quadrotor.update(delta)
    states = deepcopy(quadrotor.true_state)
    # flip the z-axis from the dynamics (it gives it back z-up)
    states.pos = np.diag([1., 1., -1.]) @ states.pos
    if SIMULATE:
        airsim_viz.update(states)

    # increment the simulation
    sim_time += SIM.ts_simulation
    # if SIMULATE:
    #     time.sleep(SIM.ts_simulation)

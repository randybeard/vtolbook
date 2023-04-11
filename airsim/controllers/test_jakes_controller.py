import time
import numpy as np
import pathlib
import sys

# this line gets the parent folder of the parent folder of the current file
# i.e. vtolbook/airsim/. If your sim file is in the airsim folder, you don't need this
directory = pathlib.Path(__file__).parent.parent
# this is so we can have the base folder of the project (vtolbook/airsim) in the path
sys.path.append(str(directory))

import parameters.quadrotor_parameters as QUAD
import parameters.simulation_parameters as SIM
from viz.quad_viewer import QuadViewer
from dynamics.quad_dynamics import QuadDynamics
from controllers.jakes_controller import Autopilot
from trajectory_generators.circular_trajectory import TrajectoryGenerator
from message_types.msg_delta import MsgDelta

airsim_viz = QuadViewer()
quadrotor = QuadDynamics(SIM.ts_simulation, QUAD)
autopilot = Autopilot(SIM.ts_simulation, QUAD)
traj_gen = TrajectoryGenerator(SIM.ts_simulation)
delta = MsgDelta()
sim_time = SIM.start_time

while sim_time < SIM.end_time:
    # get the desired trajectory from the trajectory generator
    trajectory = traj_gen.update()

    # get the estimated state (the estimate is ground truth here)
    estimated_state = quadrotor.true_state

    # get the commands from the controller
    delta, commanded_states = autopilot.update(trajectory, estimated_state)

    # update the dynamics and simulation
    quadrotor.update(delta)
    airsim_viz.update(quadrotor.true_state)

    # increment the simulation
    sim_time += SIM.ts_simulation
    time.sleep(SIM.ts_simulation)

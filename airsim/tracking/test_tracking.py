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
from controllers.jakes_controller import Autopilot
from trajectory_generators.circular_trajectory import TrajectoryGenerator
from message_types.msg_delta import MsgDelta


airsim_viz = QuadViewer()
quadrotor = QuadDynamics(SIM.ts_simulation, QUAD)
autopilot = Autopilot(SIM.ts_simulation, QUAD)
traj_gen = TrajectoryGenerator(SIM.ts_simulation)
delta = MsgDelta()
sim_time = SIM.start_time

airsim_viz.spawn_target()

while sim_time < SIM.end_time:
    airsim_viz.update_target([1,0,0],SIM.ts_simulation)

    img = airsim_viz.get_image("forward")
    cv2.imshow("forward", img)
    cv2.waitKey(1)
    
    
    # increment the simulation
    sim_time += SIM.ts_simulation
    time.sleep(SIM.ts_simulation)

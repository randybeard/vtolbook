import airsim
import math
import time
import sys
import numpy as np
sys.path.insert(0,"../..")

import parameters.quadrotor_parameters as QUAD
import parameters.simulation_parameters as SIM
from viz.quad_viewer import QuadViewer
from dynamics.quad_dynamics import QuadDynamics
from message_types.msg_delta import MsgDelta

airsim_viz = QuadViewer()
quadrotor = QuadDynamics(SIM.ts_simulation, QUAD)
delta = MsgDelta()
sim_time = SIM.start_time


while sim_time < SIM.end_time:

    delta.force = QUAD.mass*QUAD.gravity + 1.0
    delta.torque = np.array([[0.], [0.], [0.5]])

    quadrotor.update(delta)
    state = quadrotor.true_state
    
    airsim_viz.update(state)

    sim_time += SIM.ts_simulation
    time.sleep(SIM.ts_simulation)

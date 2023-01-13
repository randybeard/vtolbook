"""
holodecksim
        6/18/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

# import viewers and video writer
from chap2.holodeck_world import holodeckWorld
from tools.rotations import euler_to_rotation

# import message types
from message_types.msg_state import msgState

# initialize messages
state = msgState()  # instantiate state message

# initialize viewers and video
holodeck_world = holodeckWorld()

# initialize the simulation time
sim_time = SIM.start_time
phi = 0
theta = 0
psi = 0
# main simulation loop
while sim_time < SIM.end_time:
    # # -------vary states to check viewer-------------
    # if sim_time < SIM.end_time/6:
    #     state.pos[0][0] += 1*SIM.ts_simulation
    # elif sim_time < 2*SIM.end_time/6:
    #     state.pos[1][0] += 1*SIM.ts_simulation
    # elif sim_time < 3*SIM.end_time/6:
    #     state.pos[2][0] -= 1*SIM.ts_simulation
    # elif sim_time < 4*SIM.end_time/6:
    #     psi += 1*SIM.ts_simulation
    # elif sim_time < 5*SIM.end_time/6:
    #     theta += 1 * SIM.ts_simulation
    # else:
    #     phi += 1 * SIM.ts_simulation
    # state.rot = euler_to_rotation(phi, theta, psi)


    # set variables directly
    state.pos = np.array([[0, 0, -10]]).T
    phi = 0 * (np.pi/180)
    theta = 0 * (np.pi/180)
    psi = 0 * (np.pi/180)
    state.rot = euler_to_rotation(phi, theta, psi)
    azimuth = 0 * (np.pi/180)
    elevation = 0 * (np.pi/180)
    state.gimbal = np.array([[azimuth, elevation, 0]]).T

    # -------update viewer and video-------------
    camera_image = holodeck_world.update(state)

    # -------increment time-------------
    sim_time += SIM.ts_simulation





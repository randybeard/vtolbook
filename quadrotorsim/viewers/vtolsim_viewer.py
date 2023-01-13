"""
quadrotorsim
        3/18/2020 - RWB
"""
import numpy as np
import sys
sys.path.append('..')

# import viewers and video writer
from viewers.vtol_viewer import VtolViewer
from tools.rotations import euler_to_rotation

# import parameters
import parameters.simulation_parameters as SIM
# import message types
from message_types.msg_state import MsgState

# initialize messages
state = MsgState()  # instantiate state message

# initialize viewers and video
vtol_view = VtolViewer()

# initialize the simulation time
sim_time = SIM.start_time
psi = 0
theta = 0
phi = 0
# main simulation loop
while sim_time < SIM.end_time:
    # -------vary states to check viewer-------------
    if sim_time < SIM.end_time/6:
        state.pos[0] += 10*SIM.ts_simulation
        state.rotors[0] = np.radians(0)
        state.rotors[1] = np.radians(45)
    elif sim_time < 2*SIM.end_time/6:
        state.pos[1] += 10*SIM.ts_simulation
        state.rotors[0] = np.radians(45)
        state.rotors[1] = np.radians(90)
    elif sim_time < 3*SIM.end_time/6:
        state.pos[2] -= 10*SIM.ts_simulation
        state.rotors[0] = np.radians(90)
        state.rotors[1] = np.radians(45)
    elif sim_time < 4*SIM.end_time/6:
        psi += 1*SIM.ts_simulation
        state.rotors[0] = np.radians(0)
        state.rotors[1] = np.radians(0)
    elif sim_time < 5*SIM.end_time/6:
        theta += 1*SIM.ts_simulation
    else:
        phi += 1*SIM.ts_simulation
    state.rot = euler_to_rotation(phi, theta, psi)

    # -------update viewer and video-------------
    vtol_view.update(state)

    # -------increment time-------------
    sim_time += SIM.ts_simulation





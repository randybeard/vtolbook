"""
vtolsim
    - Update history:  
        5/4/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# import viewers and video writer
from viewers.vtol_viewer import vtolViewer

# import parameters
import parameters.simulation_parameters as SIM
# import message types
from message_types.msg_state import msgState

# initialize messages
state = msgState()  # instantiate state message

# initialize viewers
vtol_view = vtolViewer()

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
while sim_time < SIM.end_time:
    #-------vary states to check viewer-------------
    if sim_time < SIM.end_time/6:
        state.pn += 10*SIM.ts_simulation
    elif sim_time < 2*SIM.end_time/6:
        state.pe += 10*SIM.ts_simulation
    elif sim_time < 3*SIM.end_time/6:
        state.h += 10*SIM.ts_simulation
    elif sim_time < 4*SIM.end_time/6:
        state.psi += 0.1*SIM.ts_simulation
    elif sim_time < 5*SIM.end_time/6:
        state.theta += 0.1*SIM.ts_simulation
    else:
        state.phi += 0.1*SIM.ts_simulation
    state.right_rotor = (np.pi/2) * np.sin(0.1*sim_time)
    state.left_rotor = (np.pi/2) * np.cos(0.1*sim_time)

    #-------update viewer and video-------------
    vtol_view.update(state)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

print("Press Ctrl-Q to exit...")




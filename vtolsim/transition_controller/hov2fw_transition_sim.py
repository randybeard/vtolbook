"""
vtolsim
    - Update history:
        5/8/2019 - R.W. Beard
"""
#/usr/bin/python3
import sys
sys.path.append('..')
sys.path.append('../viewers')
import numpy as np
import parameters.simulation_parameters as SIM

from viewers.vtol_viewer import vtolViewer
from viewers.state_viewer import stateViewer
from viewers.actuators_viewer import actuatorsViewer
from viewers.hl_controls_viewer import HLControlsViewer
from dynamics.vtol_dynamics import vtolDynamics
from message_types.msg_controls import msgControls
from dynamics.trim import *
from low_level_controller.low_level_control import lowLevelControl
from tools.msg_convert import *
from tools.rotations import Quaternion2Euler

np.set_printoptions(precision=4, linewidth=200, suppress=True)
# initialize viewers
vtol_view = vtolViewer()
state_view = stateViewer()
actuators_view = actuatorsViewer()
hl_controls_view = HLControlsViewer()

# initialize elements of the architecture
vtol = vtolDynamics()

#initialize low level control
low_ctrl = lowLevelControl(M=0.5, Va0=2.0, ts_control=SIM.ts_simulation)


# calculate_trim for nominal **hover** conditions
Va_star = 0.0
gamma_star = 0.0
state_trim, delta_trim = compute_trim(vtol, Va_star, gamma_star, servo0 = np.radians(90))
vtol._state = state_trim
vtol._update_true_state()


# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation

trans_t_start = 5.0
trans_t_len = 3.0

sig_i = 0.0
sig_e = 1.0
sig = 0.0

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol._state  # estimated state is current state

    # if sim_time > trans_t_start and sim_time < (trans_t_start + trans_t_len):
    #     sig += Ts*(sig_e - sig_i)/(trans_t_len)
    #     print("sig = ", sig)
    # elif sim_time > (trans_t_start + trans_t_len):
    #     sig = sig_e

    #------- Low Level Controller -------------
    # LL controller inputs:
    # u =  ax, ay, tau_x, tau_y, tau_z
    u = np.array([[0., .8, 0., 0., 0.]]).T
    delta_msg = low_ctrl.update(u[2:5], u[0:2], vtol.true_state)

    #-------update physical system-------------
    # delta_msg = np3msg_controls(delta);
    vtol.update(delta_msg, np.zeros((6,1)))  # propagate the MAV dynamics

    #-------update viewers-------------
    vtol_view.update(vtol.true_state)
    state_view.update(vtol._state, vtol._state, vtol._state, u[2:], Ts)
    actuators_view.update(delta_msg, vtol._state[13:15], Ts)
    hl_controls_view.update(u, np.zeros(5), Ts)

    #-------increment time-------------
    sim_time += Ts

# wait for user input to close
input()


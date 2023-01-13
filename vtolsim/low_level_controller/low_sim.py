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
from viewers.data_viewer import dataViewer
from viewers.controls_viewer import controlsViewer
from dynamics.vtol_dynamics import vtolDynamics
from dynamics.wind_simulation import windSimulation
from message_types.msg_controls import msgControls
from dynamics.compute_models import compute_tf_model
from dynamics.trim import *
from low_level_controller.low_level_control import lowLevelControl
from low_level_controller.attitude_control import attitudeControl
from tools.signals import signals
import time

# initialize viewers
vtol_view = vtolViewer()
data_view = dataViewer()
controls_view = controlsViewer()

# initialize elements of the architecture
wind = windSimulation()
vtol = vtolDynamics()

# initialize command message
delta = msgControls()

#initialize low level control
att_ctrl = attitudeControl(ts_control=SIM.ts_simulation)
low_ctrl = lowLevelControl(M=0.5, Va0=15.0, ts_control=SIM.ts_simulation)

#set desired trim params
Va_desired = 0.0
gamma_desired = np.radians(0.0)

#calculate_trim
# state_trim, delta_trim = compute_trim(vtol, Va_desired, gamma_desired)
# value from validate forces test
#delta_trim = np.array([[0.0],
#    [0.0],
#    [0.77495385],
#    [0.6446103 ],
#    [0.64774539],
#    [1.53628205],
#    [1.60514349]])
# print("new trim")
# print(delta_trim)

# compute_tf_model(vtol, state_trim, delta_trim)
# vtol._state = state_trim
vtol._update_true_state()
vtol._update_velocity_data()
delta.elevon_right = 0.0#delta_trim.item(0)
delta.elevon_left = 0.0#delta_trim.item(1)
delta.throttle_rear = 0.0#delta_trim.item(2)
delta.throttle_right = 0.0#delta_trim.item(3)
delta.throttle_left = 0.0#delta_trim.item(4)
delta.servo_right = 0.0#delta_trim.item(5)
delta.servo_left = 0.0#delta_trim.item(6)

p_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
q_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
r_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)

phi_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
theta_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
psi_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)

# print(vtol._forces_moments(delta))
# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation
# vtol._state[13:15] = 45.0*np.pi/180.0

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol.true_state  # estimated state is current state

    #-------controller-------------
    commanded_state = vtol.true_state  # commanded state is current state

    # att_cmd = np.array([[phi_command.square(sim_time)],
    att_cmd = np.array([[0.],
            [0.0],#theta_command.square(sim_time)],
            [0.0]])#psi_command.square(sim_time)]])

    omega_d = att_ctrl.update(att_cmd, vtol.true_state)
    Tz = .01*(-.1*np.clip(estimated_state.h, -10., 10.)) + .8394242
    Tz = np.clip(Tz, 0., 1.)
    print(Tz)
    delta = low_ctrl.update(omega_d, np.array([[0.0],[Tz]]), vtol.true_state)#np.array([[p_command.square(sim_time)],[0.0],[0.0]])

    #-------update physical system-------------
    current_wind = wind.update()  # get the new wind vector
    vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics

    #-------update viewers-------------
    vtol_view.update(vtol.true_state)
    data_view.update(vtol.true_state, estimated_state, commanded_state, Ts)
    controls_view.update(delta, Ts)

    #-------increment time-------------
    sim_time += Ts

input("Press a key to exit")

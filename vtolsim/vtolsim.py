"""
vtolsim
    - Update history:
        5/8/2019 - R.W. Beard
        2/8/2021 - Thane Downing
"""
#/usr/bin/python3
import sys
sys.path.append('..')
sys.path.append('./viewers')
import numpy as np
import parameters.simulation_parameters as SIM

from viewers.vtol_viewer import vtolViewer
from viewers.data_viewer import dataViewer
from viewers.airsim_demo import airsimDemo
from viewers.controls_viewer import controlsViewer
from dynamics.vtol_dynamics import vtolDynamics
from dynamics.wind_simulation import windSimulation
from message_types.msg_controls import msgControls
from dynamics.trim import *

from time import monotonic as now

# initialize viewers
vtol_view = vtolViewer()
data_view = dataViewer()
airsim_demo = airsimDemo()
controls_view = controlsViewer()

# initialize elements of the architecture
wind = windSimulation()
vtol = vtolDynamics()

# initialize command message
delta = msgControls()

#set desired trim params
Va_desired = 0.0
gamma_desired = np.radians(0.0)

#calculate_trim
state_trim, delta_trim = compute_trim(vtol, Va_desired, gamma_desired, servo0=np.pi/2)

vtol._state = state_trim
vtol._update_true_state()
vtol._update_velocity_data()
delta.elevon_right = 0.0#delta_trim.item(0)
delta.elevon_left = 0.0#delta_trim.item(1)
delta.throttle_rear = delta_trim.item(2)
delta.throttle_right = delta_trim.item(3)
delta.throttle_left = delta_trim.item(4)
delta.servo_right = delta_trim.item(5)
delta.servo_left = delta_trim.item(6)

print(vtol._forces_moments(delta))
# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation

# main simulation loop
print("Press Command-Q (Ctrl-C) to exit...")
while sim_time < SIM.end_time:
    start_time = now()
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol.true_state  # estimated state is current state

    #-------controller-------------
    commanded_state = vtol.true_state  # commanded state is current state

    #-------update physical system-------------
    current_wind = wind.update()  # get the new wind vector
    vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics

    #-------update viewers-------------
    vtol_view.update(vtol.true_state)
    data_view.update(vtol.true_state, estimated_state, commanded_state, Ts)
    airsim_demo.update(vtol.true_state)

    controls_view.update(delta, Ts)

    #-------increment time-------------
    sim_time += Ts
    end_time = now()
    sim_rate = 1/(end_time-start_time)
    print("Simulation Rate {:.3f}fps".format(sim_rate), end = '\r')

input("Press a key to exit")

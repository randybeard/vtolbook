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
from low_level_controller.rate_control import RateControl
from low_level_controller.attitude_control import attitudeControl
from control_allocation.control_allocation_cls import ControlAllocation
from tools.signals import signals
from tools.msg_convert import *
import time

# initialize viewers
vtol_view = vtolViewer()
data_view = dataViewer()
controls_view = controlsViewer()

# initialize elements of the architecture
wind = windSimulation()
vtol = vtolDynamics()

# trim
Va_star = 0.0
gamma_star = 0.0
servo0 = np.radians(90)
state_trim, delta_trim = compute_trim(vtol, Va_star, gamma_star, servo0 = np.radians(90))
vtol._state = state_trim
vtol._update_true_state()

#initialize low level control
att_ctrl = attitudeControl(ts_control=SIM.ts_simulation)
rate_control = RateControl(ts_control=SIM.ts_simulation)
control_alloc = ControlAllocation(servo0=servo0)

# signal generators
p_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
q_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
r_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)

phi_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
theta_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
psi_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)

# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol.true_state  # estimated state is current state

    #-------controller-------------
    commanded_state = vtol.true_state  # commanded state is current state

    att_cmd = np.array(
        [0., #np.array([phi_command.square(sim_time)
        0.0, #theta_command.square(sim_time),
        0.0] #psi_command.square(sim_time)]
    )

    omega_d = att_ctrl.update(att_cmd, vtol.true_state).reshape(-1)
    omega = vtol._state[10:13]
    tau_d = rate_control.update(omega_d, omega)
    delta = control_alloc.update(np.array([0., -9.81]), tau_d, vtol._Va)

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

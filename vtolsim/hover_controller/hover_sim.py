"""
vtolsim_python
    - Chapter 6 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/5/2019 - RWB
"""
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
from dynamics.trim import compute_trim
from hover_controller.hover_autopilot import hoverController
from message_types.msg_controls import msgControls
from tools.signals import signals

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
vtol_view = vtolViewer()  # initialize the vtol viewer
data_view = dataViewer()  # initialize view of data plots
controls_view = controlsViewer()

delta = msgControls()

if VIDEO == True:
    from viewers.video_writer import videoWriter
    video = videoWriter(video_name="chap6_video.avi",
                         bounding_box=(0, 0, 1000, 1000),
                         output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = windSimulation()
vtol = vtolDynamics()
ctrl = hoverController(SIM.ts_simulation)

# autopilot commands
from message_types.msg_autopilot import msgAutopilot
commands = msgAutopilot()
# Va_command = signals(dc_offset=25.0, amplitude=3.0, start_time=2.0, frequency = 0.01)
# h_command = signals(dc_offset=100.0, amplitude=10.0, start_time=0.0, frequency = 0.02)
# chi_command = signals(dc_offset=np.radians(180), amplitude=np.radians(45), start_time=5.0, frequency = 0.015)

phi_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.1)
theta_command = signals(dc_offset=np.radians(0), amplitude=np.radians(15), start_time=0.0, frequency = 0.05)
psi_command = signals(dc_offset=np.radians(0), amplitude=np.radians(20), start_time=0.0, frequency = 0.025)
h_command = signals(dc_offset=100, amplitude=5, start_time=5.0, frequency = 0.025)
pn_command = signals(dc_offset=0, amplitude=5, start_time=10.0, frequency = 0.025)
pe_command = signals(dc_offset=0, amplitude=5, start_time=0.0, frequency = 0.025)

Va_star = 0.0
gamma_star = 0.0
# state_trim, delta_trim = compute_trim(vtol, Va_star, gamma_star)
state_trim = np.array([[-1.04239742e-12],  [3.06873064e-12], [-1.00000000e+02], [-5.94131919e-07],
   [0.00000000e+00],  [1.86190987e-06],  [9.99985895e-01],  [0.00000000e+00],
   [5.31122783e-03],  [0.00000000e+00],  [0.00000000e+00],  [0.00000000e+00],
   [0.00000000e+00],  [1.51897067e+00],  [1.59062520e+00]])

delta_trim = np.array([[-0.05454118],  [0.06484209],  [0.78348849],  [0.64704488],  [0.64844094],  [1.51897067],
   [1.5906252]])


vtol._state = state_trim
vtol._update_true_state()
delta.elevon_right = delta_trim.item(0)
delta.elevon_left = delta_trim.item(1)
delta.throttle_rear = delta_trim.item(2)
delta.throttle_right = delta_trim.item(3)
delta.throttle_left = delta_trim.item(4)
delta.servo_right = delta_trim.item(5)
delta.servo_left = delta_trim.item(6)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    #-------autopilot commands-------------
    commands.roll_command = 0.0#phi_command.square(sim_time)
    commands.pitch_command = 0.0#theta_command.square(sim_time)
    commands.yaw_command = 0.0#psi_command.square(sim_time)
    commands.altitude_command = h_command.square(sim_time)
    commands.pn_command = pn_command.square(sim_time)
    commands.pe_command = pe_command.square(sim_time)


    #-------controller-------------
    estimated_state = vtol.true_state  # uses true states in the control
    delta, commanded_state = ctrl.update(commands, estimated_state)

    #-------physical system-------------
    current_wind = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])#wind.update()  # get the new wind vector
    vtol.update(delta, current_wind)  # propagate the vtol dynamics

    #-------update viewer-------------
    vtol_view.update(vtol.true_state)  # plot body of vtol
    data_view.update(vtol.true_state, # true states
                     estimated_state, # estimated states
                     commanded_state, # commanded states
                     SIM.ts_simulation)
    controls_view.update(delta, SIM.ts_simulation)
    if VIDEO == True: video.update(sim_time)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO == True: video.close()
input("Press a key to exit")

"""
holodecksim
        6/24/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap2.holodeck_world import holodeckWorld
from chap3.data_viewer import dataViewer
from chap3.multirotor_dynamics import multirotorDynamics
from chap4.autopilot import autopilot
from tools.signals import signals

# initialize holodeck and visualization
VIDEO = False  # True==write video, False==don't write video
holodeck_world = holodeckWorld()  # initialize holodeck
data_view = dataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import videoWriter
    video = videoWriter(video_name="chap4_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
multirotor = multirotorDynamics(SIM.ts_simulation)
ctrl = autopilot(SIM.ts_simulation)

# autopilot commands
from message_types.msg_autopilot import msgAutopilot
commands = msgAutopilot()

#
# Va_command = signals(dc_offset=25.0,
#                      amplitude=3.0,
#                      start_time=2.0,
#                      frequency=0.01)
# h_command = signals(dc_offset=100.0,
#                     amplitude=10.0,
#                     start_time=0.0,
#                     frequency=0.02)
# chi_command = signals(dc_offset=np.radians(180),
#                       amplitude=np.radians(45),
#                       start_time=5.0,
#                       frequency=0.015)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    # -------autopilot commands-------------
    commands.vel = np.array([[0.0, 1.0, -1.0]]).T
    commands.psi = np.pi / 2.
    # commands.omega = np.array([[0.], [0.], [0.]])
    # commands.omega_dot = np.array([[0.], [0.], [0.]])
    # commands.airspeed_command = Va_command.square(sim_time)
    # commands.course_command = chi_command.square(sim_time)
    # commands.altitude_command = h_command.square(sim_time)

    # -------controller-------------
    estimated_state = multirotor.true_state  # uses true states in the control
    delta, commanded_state = ctrl.update(commands, estimated_state)

    # -------update physical system-------------
    multirotor.update(delta)
    camera_image = holodeck_world.update(estimated_state)

    # -------update viewer-------------
    data_view.update(multirotor.true_state,  # true states
                     estimated_state,  # estimated states
                     commanded_state,  # commanded states
                     delta,  # inputs to the multirotor
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




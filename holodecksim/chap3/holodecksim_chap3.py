"""
holodecksim
        3/18/2020 - RWB
        6/23/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.multirotor_parameters as SYS


from chap2.holodeck_world import holodeckWorld
from chap3.data_viewer import dataViewer
from chap3.multirotor_dynamics import multirotorDynamics
from message_types.msg_delta import msgDelta

# initialize holodeck and visualization
VIDEO = False  # True==write video, False==don't write video
holodeck_world = holodeckWorld()  # initialize holodeck
data_view = dataViewer()  # initialize view of data plots
if VIDEO is True:
    from chap2.video_writer import videoWriter
    video = videoWriter(video_name="chap3_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
multirotor = multirotorDynamics(SIM.ts_simulation)
delta = msgDelta()

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:

    # -------vary forces and moments to check dynamics-------------
    delta.force = SYS.mass * SYS.gravity + 1.0
    delta.torque = np.array([[0.], [0.], [0.]])
    delta.gimbal_input = np.array([[0.], [0.], [0.]])

    # -------update physical system-------------
    multirotor.update(delta)
    camera_image = holodeck_world.update(multirotor.true_state)

    # -------update viewer-------------
    data_view.update(multirotor.true_state,  # true states
                     multirotor.true_state,  # estimated states
                     multirotor.true_state,  # commanded states
                     delta,  # inputs to the quadrotor
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()






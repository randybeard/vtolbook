"""
mavsim_python
    - Chapter 12 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        4/3/2019 - BGM
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN

from chap3.data_viewer import dataViewer
from chap4.wind_simulation import windSimulation
from chap6.autopilot import autopilot
from chap7.mav_dynamics import mavDynamics
from chap8.observer import observer
from chap10.path_follower import pathFollower
from chap11.path_manager import pathManager
from chap12.world_viewer import worldViewer
from chap12.path_planner import pathPlanner

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
world_view = worldViewer()  # initialize the viewer
data_view = dataViewer()  # initialize view of data plots
if VIDEO == True:
    from chap2.video_writer import videoVriter
    video = videoWriter(video_name="chap12_video.avi",
                         bounding_box=(0, 0, 1000, 1000),
                         output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = windSimulation(SIM.ts_simulation)
mav = mavDynamics(SIM.ts_simulation)
ctrl = autopilot(SIM.ts_simulation)
obsv = observer(SIM.ts_simulation)
path_follow = pathFollower()
path_manage = pathManager()
path_plan = pathPlanner()

from message_types.msg_map import msgMap
map = msgMap()


# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = obsv.update(measurements)  # estimate states from measurements

    # -------path planner - ----
    if path_manage.flag_need_new_waypoints is True:
        waypoints = path_plan.update(map, estimated_state, PLAN.R_min)

    #-------path manager-------------
    path = path_manage.update(waypoints, PLAN.R_min, estimated_state)

    #-------path follower-------------
    autopilot_commands = path_follow.update(path, estimated_state)

    #-------controller-------------
    delta, commanded_state = ctrl.update(autopilot_commands, estimated_state)

    #-------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    #-------update viewer-------------
    world_view.update(mav.true_state, path, waypoints, map)  # plot path and MAV
    data_view.update(mav.true_state, # true states
                     estimated_state, # estimated states
                     commanded_state, # commanded states
                     SIM.ts_simulation)
    if VIDEO == True: video.update(sim_time)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO == True: video.close()





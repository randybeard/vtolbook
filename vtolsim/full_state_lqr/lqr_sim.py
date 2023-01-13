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
from dynamics.wind_simulation import windSimulation
from message_types.msg_controls import msgControls
from full_state_lqr import lqr_control
from dynamics.trim import *
from low_level_controller.low_level_control import lowLevelControl
from tools.msg_convert import *
from tools.rotations import Quaternion2Euler
from trajectory_planning.trajectory import XYZSinusoid, HANDTraj
from trajectory_planning.spline_trajectory import SplineTrajectory
import parameters.spline_parameters as SPLP
from trajectory_planning.differential_flatness import DifferentialFlatness

np.set_printoptions(precision=4, linewidth=200, suppress=True)
# initialize viewers
vtol_view = vtolViewer()
state_view = stateViewer()
actuators_view = actuatorsViewer()
hl_controls_view = HLControlsViewer()

# initialize elements of the architecture
wind = windSimulation()
vtol = vtolDynamics()

# initialize trajectory
# df_traj = DifferentialFlatness(XYZSinusoid(150., 150., 75., 600., 300., 600., -np.pi/2, np.pi, 0.*np.pi/2))
# df_traj = DifferentialFlatness(HANDTraj())
df_traj = DifferentialFlatness(SplineTrajectory(SPLP.pts, SPLP.vels)) # points, max velocity
step =  0.*df_traj.traj.s_max/500
vtol_view.addTrajectory(df_traj.traj.getPList(df_traj.traj.getP, 0., df_traj.traj.s_max + step, df_traj.traj.s_max/500))
#initialize low level control
low_ctrl = lowLevelControl(M=0.5, Va0=2.0, ts_control=SIM.ts_simulation)

# initialize command message
delta = msgControls()

#calculate_trim
vtol._update_true_state()

# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation

u_prev = np.zeros((5,1))

stuff_to_save = np.zeros((13,1))

# main simulation loop
print("Press Command-Q to exit...")
alpha = .5
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol._state  # estimated state is current state

    # ------ Trajectory follower
    desired_state, desired_input = df_traj.desiredState_fromX(estimated_state[0:10])

    #------- High Level controller-------------

    u = lqr_control.update(estimated_state[0:10], desired_state, u_prev, desired_input, df_traj)
    u = alpha*u_prev + (1-alpha)*u
    u_prev = u

    f_mag = np.linalg.norm(u[0:2])
    if(f_mag > .9):
        u[0:2] = .9*u[0:2] / f_mag

    if(u.item(2) > 1.0):
        u[2] = 1.0
    if(u.item(3) > 1.0):
        u[3] = 1.0
    if(u.item(4) > 1.0):
        u[4] = 1.0

    if(u.item(0) < 0.0):
        u[0] = 0.0
    if(u.item(1) < 0.0):
        u[1] = 0.0
    if(u.item(2) < -1.0):
        u[2] = -1.0
    if(u.item(3) < -1.0):
        u[3] = -1.0
    if(u.item(4) < -1.0):
        u[4] = -1.0
    # print("u = ", u.T)

    #------- Low Level Controller -------------
    delta = low_ctrl.update(u[2:5], u[0:2], vtol.true_state)

    #-------update physical system-------------
    current_wind = wind.update()  # get the new wind vector
    # delta_msg = np2msg_controls(delta_trim)
    vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics

    #-------update viewers-------------
    vtol_view.update(vtol.true_state)
    state_view.update(vtol._state, estimated_state, desired_state, u[2:], Ts)
    actuators_view.update(delta, vtol._state[13:15], Ts)
    hl_controls_view.update(u, desired_input, Ts)

    #-------add stuff to save----------
    x = vtol.true_state
    stuff_to_save = np.append(stuff_to_save, np.array([[sim_time],[x.pn],[x.pe],[x.h],[x.u],[x.v],[x.w], 
        [desired_state.item(0)], [desired_state.item(1)], [-desired_state.item(2)], [desired_state.item(3)], [desired_state.item(4)], [desired_state.item(5)]]), axis=1)


    #-------increment time-------------
    sim_time += Ts

fn = input("Give me a file name (including .csv)!! X to cancel\n")
if fn != "X":
    np.savetxt(fn, np.delete(stuff_to_save, 0, 1), delimiter=', ', header="t, pn, pe, h, u, v, w, pn_d, pe_d, h_d, u_d, v_d, w_d")


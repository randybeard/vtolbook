"""
vtolsim
    - Update history:
        5/8/2019 - R.W. Beard
"""
#/usr/bin/python3
import sys
sys.path.append('..')
sys.path.append('../viewers')
sys.path.append('../trajectorygenerator/scripts')
import numpy as np
import matplotlib.pyplot as plt
import time
import parameters.simulation_parameters as SIM

from viewers.state_viewer import stateViewer
from dynamics.vtol_dynamics import vtolDynamics
from dynamics.wind_simulation import windSimulation
from message_types.msg_controls import msgControls
from dynamics.trim import *
from control_allocation.control_allocation_cls import ControlAllocation
from low_level_controller.rate_control import RateControl
from geometric_control.geometric_controller import GeometricController
from geometric_control.geometric_zthrust_controller import GeometricZThrustController
from tools.msg_convert import *
from tools.rotations import Quaternion2Euler, Rotation2Quaternion, Rotation2Euler

# trajectories
from bspline_trajectory_generator import BSplineTrajectoryGenerator
from geometric_control.random_path import RandomPath
from trajectory_plotter import plotVsTime, evalVector

SHOW_STATE_VIEW = False

def path2d_to_path4d(path2d, zmax):
    if path2d.shape[0] < 2*zmax:
        print(f"Error creating path - zmax ({zmax}) must be at least 2x path length ({path2d.shape[0]})")
        exit()

    zpath = np.concatenate([
        np.arange(0, zmax, 1),
        zmax * np.ones(path2d.shape[0] - 2*zmax),
        np.arange(zmax-1, -1, -1)
    ])

    yawpath = np.zeros(path2d.shape[0])

    return 2.*np.column_stack([path2d, zpath, yawpath])

def sim_loop(random_path_generator, npath_points=50, zmax=5):
    np.set_printoptions(precision=4, linewidth=200, suppress=True)

    if SHOW_STATE_VIEW:
        state_view = stateViewer()

    # initialize elements of the architecture
    wind = windSimulation()
    vtol = vtolDynamics()

    # setup trajectory
    path2d = random_path_generator.generate(start_loc = np.array([0, 0]), npoints=npath_points, start_dir=3, maintain_dir=True)

    path4d = path2d_to_path4d(path2d, zmax)

    BSTrajGen = BSplineTrajectoryGenerator()

    bspline_traj = BSTrajGen.generate_trajectory(path4d.T, 4)

    sim_trajectory = bspline_traj; traj_name = "MonteCarloBSpline"; SIM.end_time = bspline_traj.members[0].total_time/bspline_traj.time_scale  + 5

    sim_trajectory_eval, _ = evalVector(sim_trajectory, 0, 0, SIM.end_time, tsteps=1000)

    # initialize geometric controller
    geom_ctrl = GeometricController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)
    # geom_ctrl = GeometricZThrustController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)

    #initialize low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    control_alloc = ControlAllocation(servo0=np.pi/2)

    # initialize command message
    delta = msgControls()

    vtol._update_true_state()

    # initialize the simulation time
    sim_time = SIM.start_time
    Ts = SIM.ts_simulation

    error_hist = []
    time_hist = []
    pos_hist = []

    # main simulation loop
    while sim_time < SIM.end_time:
        #-------observer-------------
        estimated_state = vtol._state  # estimated state is current state

        ctrl_start_time = time.time()
        # ------ Trajectory follower
        traj_derivatives_at_t = sim_trajectory.evalUpToKr(sim_time, 4)

        #------- High Level controller-------------
        T, R_d, omega_c, pd_i, vd_b = geom_ctrl.update(estimated_state[0:10], traj_derivatives_at_t)

        #------- Low Level Controller -------------
        omega = estimated_state[10:13,0]
        tau_c = rate_control.update(omega_c, omega)
        delta = control_alloc.update(T, tau_c, vtol._Va)
        # delta = control_alloc.update_lp(T, tau_c, vtol._Va)

        ctrl_end_time = time.time()
        #-------update physical system-------------
        current_wind = wind.update()  # get the new wind vector
        vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics
        # vtol.update(delta, current_wind)  # propagate the MAV dynamics

        #-------update viewers-------------

        desired_state = np.concatenate([
            pd_i,
            vd_b,
            Rotation2Quaternion(R_d).reshape(-1),
            omega_c]).reshape((-1, 1))

        if SHOW_STATE_VIEW:
            state_view.update(vtol._state, estimated_state, desired_state, omega_c, Ts)

        error_hist.append(np.linalg.norm(pd_i - vtol._state[0:3,:].reshape(-1)))
        time_hist.append(ctrl_end_time - ctrl_start_time)
        pos_hist.append(np.copy(vtol._state[0:3,:].reshape(-1)))

        #-------increment time-------------
        sim_time += Ts

    error_hist = np.array(error_hist)

    error_avg = np.mean(error_hist)
    error_max = np.max(error_hist)

    time_avg = np.mean(time_hist)
    time_max = np.max(time_hist)

    pos_hist = np.array(pos_hist)

    return error_avg, error_max, time_avg, time_max, pos_hist

def run_multiple_sims(nsims = 3):
    random_path_generator = RandomPath(0)

    trajectory_hist = []
    error_avg_hist = []
    error_max_hist = []
    time_avg_hist = []
    time_max_hist = []

    for i in range(nsims):
        error_avg, error_max, time_avg, time_max, traj = sim_loop(random_path_generator, npath_points=51, zmax=5)
        print(f"Done with simulation run {i}")
        print(f"error_avg = {error_avg}, \t error_max = {error_max}")
        trajectory_hist.append(traj)
        error_avg_hist.append(error_avg)
        error_max_hist.append(error_max)
        time_avg_hist.append(time_avg)
        time_max_hist.append(time_max)

    hist_fname = f"sim_data/MonteCarlo_{time.strftime('%d%b%Y_%H:%M:%S')}.npz"
    np.savez(
        hist_fname,
        trajectories=np.array(trajectory_hist),
        error_avg=np.array(error_avg_hist),
        error_max=np.array(error_max_hist),
        time_avg=np.array(time_avg_hist),
        time_max=np.array(time_max_hist),
    )


if __name__ == "__main__":
    run_multiple_sims(200)

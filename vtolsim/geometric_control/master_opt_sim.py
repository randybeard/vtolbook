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
import time
import parameters.simulation_parameters as SIM

from viewers.vtol_viewer import vtolViewer
from viewers.state_viewer import stateViewer
from viewers.actuators_viewer import actuatorsViewer
from viewers.thrust_torque_viewer import ThrustTorqueViewer
from dynamics.vtol_dynamics import vtolDynamics
from dynamics.wind_simulation import windSimulation
from message_types.msg_controls import msgControls
from dynamics.trim import *
from control_allocation.control_allocation_cls import ControlAllocation
from control_allocation.optimal_control_allocation import OptimalControlAllocation
from low_level_controller.rate_control import RateControl
from geometric_control.geometric_controller import GeometricController
from geometric_control.geometric_zthrust_controller import GeometricZThrustController
from tools.msg_convert import *
from tools.rotations import Quaternion2Euler, Rotation2Quaternion, Rotation2Euler

if SIM.show_airsim:
    from viewers.airsim_demo import airsimDemo

# trajectories
from trajectory import Trajectory
from sinusoidal_trajectory_member import SinusoidalTrajectoryMember
from linear_trajectory_member import LinearTrajectoryMember
from quadratic_trajectory_member import QuadraticTrajectoryMember
from trajectory_planning.takeoff_coast_land_trajectory import TakeoffCoastLandTrajectory
from setup_polynomial_trajectory import setup_polynomial_acc_2021_tcl_trajectory, setup_polynomial_airsim_demo_tcl_trajectory, setup_bspline_airsim_demo_trajectory
import trajectory_plotter


def main():
    np.set_printoptions(precision=4, linewidth=200, suppress=True)
    # initialize viewers
    actuators_view = actuatorsViewer()
    thrust_torque_view = ThrustTorqueViewer()
    state_view = stateViewer()
    vtol_view = vtolViewer()
    if SIM.show_airsim:
        airsim_demo = airsimDemo()

    # initialize elements of the architecture
    wind = windSimulation()
    vtol = vtolDynamics()

    # INITIALIZE TRAJECTORIES

    # 5m radius circular orbit in i-j plane at 10m alt.
    # centered at 0, -5
    # yaw pointing along trajectory
    circle_Pn_traj_memb = SinusoidalTrajectoryMember(0., 5., 30., 0.)
    circle_Pe_traj_memb = SinusoidalTrajectoryMember(-5., 5., 30., np.pi/2)
    circle_Pd_traj_memb = SinusoidalTrajectoryMember(-2., 2.0, 30., np.pi/2)
    circle_Psi_traj_memb = LinearTrajectoryMember(0.0, -2*np.pi/30.)

    circle_trajectory = Trajectory(
        [circle_Pn_traj_memb,
            circle_Pe_traj_memb,
            circle_Pd_traj_memb,
            circle_Psi_traj_memb],
        time_scale = 1.
    )

    pringle_Pn_traj_memb = SinusoidalTrajectoryMember(0., 5., 30., 0.)
    pringle_Pe_traj_memb = SinusoidalTrajectoryMember(-5., 5., 15., 0.)
    pringle_Pd_traj_memb = SinusoidalTrajectoryMember(10., 5.0, 30., np.pi/2)
    pringle_Psi_traj_memb = LinearTrajectoryMember(0.0, -2*np.pi/30.)

    pringle_trajectory = Trajectory(
        [pringle_Pn_traj_memb,
            pringle_Pe_traj_memb,
            pringle_Pd_traj_memb,
            pringle_Psi_traj_memb],
        time_scale = .5
    )

    line_Pn_traj_memb = QuadraticTrajectoryMember(.4, 0., 0.) # Pn = .5*.5*t**2
    line_Pe_traj_memb = LinearTrajectoryMember(0.0, 0.0)
    line_Pd_traj_memb = LinearTrajectoryMember(0.0, 0.)
    line_Psi_traj_memb = LinearTrajectoryMember(0.0, 0.0)

    line_trajectory = Trajectory(
        [line_Pn_traj_memb,
            line_Pe_traj_memb,
            line_Pd_traj_memb,
            line_Psi_traj_memb],
    )

    tcl_trajectory = TakeoffCoastLandTrajectory(
        10, # t_takeoff
        10, # t_coast
        15, # t_land
        10, # max_altitude
        5) # max_horizontal_velocity

    acc_poly_tcl_trajectory = setup_polynomial_acc_2021_tcl_trajectory(.2, optimize_segment_times=False)
    airsim_poly_tcl_trajectory_nopt = setup_polynomial_airsim_demo_tcl_trajectory(.2, optimize_segment_times=False)
    airsim_poly_tcl_trajectory_opt = setup_polynomial_airsim_demo_tcl_trajectory(.2, optimize_segment_times=True)
    airsim_bspline_tcl_trajectory = setup_bspline_airsim_demo_trajectory(.4)
    bspline_time_scale = airsim_bspline_tcl_trajectory.members[0].total_time*.2/airsim_poly_tcl_trajectory_opt.members[0].total_time
    airsim_bspline_tcl_trajectory.time_scale = bspline_time_scale


    ## ---------------------------------
    #  Edit here to change the trajectory that gets used

    # sim_trajectory = circle_trajectory; traj_name = "Circle"; SIM.end_time = 30.
    # sim_trajectory = pringle_trajectory; traj_name = "Pringle"
    # sim_trajectory = line_trajectory; traj_name = "Line"
    # sim_trajectory = tcl_trajectory; traj_name = "TCL"; SIM.end_time = tcl_trajectory.t6
    sim_trajectory = acc_poly_tcl_trajectory; traj_name = "ACCPolyTCL"; SIM.end_time = acc_poly_tcl_trajectory.members[0].num_segments/acc_poly_tcl_trajectory.time_scale  + 5
    # sim_trajectory = airsim_poly_tcl_trajectory_opt; traj_name = "AirSimPolyTCLOpt"; SIM.end_time = airsim_poly_tcl_trajectory_opt.members[0].num_segments/airsim_poly_tcl_trajectory_opt.time_scale  + 5
    # sim_trajectory = airsim_poly_tcl_trajectory_nopt; traj_name = "AirSimPolyTCLNOpt"; SIM.end_time = airsim_poly_tcl_trajectory_nopt.members[0].num_segments/airsim_poly_tcl_trajectory_nopt.time_scale  + 5
    # sim_trajectory = airsim_bspline_tcl_trajectory; traj_name = "AirSimBSplineTCL"; SIM.end_time = airsim_bspline_tcl_trajectory.members[0].total_time/airsim_bspline_tcl_trajectory.time_scale  + 5

    ## ---------------------------------

    # draw the trajectory
    npts = int((SIM.end_time - SIM.start_time)/.01)
    trajectory_position_points, _ = trajectory_plotter.evalVector(sim_trajectory, 0, SIM.start_time, SIM.end_time, npts)

    vtol_view.addTrajectory(trajectory_position_points[:3,:])

    # initialize geometric controller
    geom_ctrl = GeometricController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)
    # geom_ctrl = GeometricZThrustController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)

    #initialize low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    # control_alloc = OptimalControlAllocation()
    control_alloc = ControlAllocation(servo0=np.pi/2)


    # initialize command message
    delta = msgControls()

    #calculate_trim
    vtol._update_true_state()

    # initialize the simulation time
    sim_time = SIM.start_time
    Ts = SIM.ts_simulation

    u_prev = np.zeros((5,1))

    time_hist = []
    comp_time_hist = []
    desired_state_hist = []
    true_state_hist = []
    desired_thrust_torque_hist = []
    true_thrust_torque_hist = []

    # main simulation loop
    while sim_time < SIM.end_time:
        #-------observer-------------
        measurements = vtol.sensors()  # get sensor measurements
        estimated_state = vtol._state  # estimated state is current state

        ctrl_start_time = time.time()
        # ------ Trajectory follower
        traj_derivatives_at_t = sim_trajectory.evalUpToKr(sim_time, 4)
        # traj_derivatives_at_t = np.zeros(16).reshape((4,4))

        #------- High Level controller-------------
        T, R_d, omega_c, pd_i, vd_b = geom_ctrl.update(estimated_state[0:10], traj_derivatives_at_t)

        #------- Low Level Controller -------------
        omega = estimated_state[10:13,0]
        tau_c = rate_control.update(omega_c, omega)
        delta = control_alloc.update(T, tau_c, vtol._Va)
        # delta = control_alloc.update(T, tau_c, estimated_state, vtol._Va)
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

        thrust_torque_d = np.concatenate([T, tau_c]).reshape((-1,1))
        thrust_torque = np.concatenate([np.copy(vtol.total_thrust[[0,2]]), np.copy(vtol.total_torque)])

        vtol_view.update(vtol.true_state)
        state_view.update(vtol._state, estimated_state, desired_state, omega_c, Ts)
        actuators_view.update(delta, vtol._state[13:15], Ts)
        thrust_torque_view.update(thrust_torque.reshape(-1,1), thrust_torque_d.reshape(-1,1), Ts)
        if SIM.show_airsim:
            airsim_demo.update(vtol.true_state)

        time_hist.append(sim_time)
        comp_time_hist.append(ctrl_end_time - ctrl_start_time)
        desired_state_hist.append(np.copy(desired_state.reshape(-1)))
        true_state_hist.append(np.copy(vtol._state.reshape(-1)))
        desired_thrust_torque_hist.append(np.copy(thrust_torque_d.reshape(-1)))
        true_thrust_torque_hist.append(np.copy(thrust_torque.reshape(-1)))

        #-------increment time-------------
        sim_time += Ts
    hist_fname = f"sim_data/{geom_ctrl.name}_{traj_name}_{time.strftime('%d%b%Y_%H:%M:%S')}.npz"
    np.savez(
        hist_fname,
        time=np.array(time_hist),
        desired_state=np.array(desired_state_hist),
        true_state=np.array(true_state_hist),
        desired_thrust_torque=np.array(desired_thrust_torque_hist),
        true_thrust_torque=np.array(true_thrust_torque_hist)
       )
    print("Min computation rate = ", 1./np.max(comp_time_hist), " idx = ", np.argmax(comp_time_hist))
    print("Avg computation rate = ", 1./np.mean(comp_time_hist))

    print("Done with simulation")
    while (True):
        vtol_view.tick()
        state_view.tick()
        actuators_view.tick()
        thrust_torque_view.tick()
    return

main()

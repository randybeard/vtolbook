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
import parameters.convergence_parameters as VTOL

from viewers.vtol_viewer import vtolViewer
from viewers.state_viewer import stateViewer
from viewers.actuators_viewer import actuatorsViewer
from viewers.thrust_torque_viewer import ThrustTorqueViewer
from dynamics.vtol_dynamics import vtolDynamics
from dynamics.wind_simulation import windSimulation
from message_types.msg_controls import msgControls
from message_types.msg_path import msgPath
from dynamics.trim import *
from control_allocation.control_allocation_cls import ControlAllocation
from control_allocation.nonlinear_control_allocation import NonlinearControlAllocation
from low_level_controller.rate_control import RateControl
from geometric_control.geometric_controller import GeometricController
from geometric_control.geometric_zthrust_controller import GeometricZThrustController
from tools.msg_convert import *
from tools.rotations import Quaternion2Euler, Rotation2Quaternion, Rotation2Euler
from rapid_landing.rapid_landing_calculator import LandingRoutine
import controller.control_manager as ctrl_manager

if SIM.show_airsim:
    from viewers.airsim_demo import airsimDemo

# trajectories
import trajectory_plotter
import utils.math_tools as mt

#argparser
import argparse

def main():
    USE_NONLINEAR_ALLOC = True #args.nonlinear_alloc
    actuators_view = actuatorsViewer()
    thrust_torque_view = ThrustTorqueViewer()
    state_view = stateViewer()
    vtol_view = vtolViewer()
    if SIM.show_airsim:
        airsim_demo = airsimDemo()

    # initialize elements of the architecture
    wind = windSimulation()
    vtol = vtolDynamics()
    landing = LandingRoutine()
    # landing.reposition_start_at_origin()
    traj_name = 'rapid_landing'

    # draw the trajectory
    npts = int((SIM.end_time - SIM.start_time)/.01)
    #trajectory_position_points, _ = trajectory_plotter.evalVector(pos_traj, 0, SIM.start_time, SIM.end_time, npts)

    #vtol_view.addTrajectory(trajectory_position_points[:3,:])

    # initialize general controller
    controller = ctrl_manager.ControlManager(controller_type=ctrl_manager.CTRL_PATH_FOLLOWER, 
        switch_type=ctrl_manager.SWITCH_RAPID_LANDING, data=[landing.transition_z])
    path = msgPath()
    path.line_origin = np.array([[0.0, 0.0, 0.0]]).T
    path.line_direction = np.array([[1.0, 0.0, 0.0]]).T
    path.airspeed = VTOL.u0
    # geom_ctrl = GeometricController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True, theta_0=VTOL.theta0)
    # geom_ctrl = GeometricZThrustController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)

    #initialize low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    if USE_NONLINEAR_ALLOC:
        control_alloc = NonlinearControlAllocation()
    else:
        control_alloc = ControlAllocation()


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
        pos_derivatives_at_t = landing.eval_derivatives_up_to_K(4, sim_time)
        yaw_derivatives_at_t = np.zeros((1, 5))
        traj_derivatives_at_t = np.concatenate((pos_derivatives_at_t, yaw_derivatives_at_t), axis=0)


        #------- High Level controller-------------
        # T, R_d, omega_c, pd_i, vd_b, omega_pip = geom_ctrl.update(estimated_state[0:10], traj_derivatives_at_t)
        delta = controller.update([path, estimated_state])

        #------- Low Level Controller -------------
        # omega = estimated_state[10:13,0]
        # tau_c = rate_control.update(omega_c, omega)

        # delta = control_alloc.update(T, tau_c, estimated_state, vtol._Va)

        ctrl_end_time = time.time()

        #-------update physical system-------------
        current_wind = wind.update()  # get the new wind vector
        vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics

        #-------update viewers-------------
        # desired_state = np.concatenate([
        #     pd_i,
        #     vd_b,
        #     Rotation2Quaternion(R_d).reshape(-1),
        #     omega_c]).reshape((-1, 1))
        omega_c = np.zeros((3,1))

        # thrust_torque_d = np.concatenate([T, tau_c]).reshape((-1,1))
        thrust_torque_d = np.zeros((5,1))
        thrust_torque = np.concatenate([np.copy(vtol.total_thrust[[0,2]]), np.copy(vtol.total_torque)])

        vtol_view.update(vtol.true_state)
        state_view.update(vtol._state, estimated_state, controller.desired_state(), omega_c, Ts)
        actuators_view.update(delta, vtol._state[13:15], Ts)
        thrust_torque_view.update(thrust_torque.reshape(-1,1), thrust_torque_d.reshape(-1,1), Ts)
        if SIM.show_airsim:
            airsim_demo.update(vtol.true_state)

        time_hist.append(sim_time)
        comp_time_hist.append(ctrl_end_time - ctrl_start_time)
        # desired_state_hist.append(np.copy(desired_state.reshape(-1)))
        true_state_hist.append(np.copy(vtol._state.reshape(-1)))
        # desired_thrust_torque_hist.append(np.copy(thrust_torque_d.reshape(-1)))
        true_thrust_torque_hist.append(np.copy(thrust_torque.reshape(-1)))

        #-------increment time-------------
        sim_time += Ts
        first_iter = False
    # hist_fname = f"sim_data/{geom_ctrl.name}_{traj_name}_{time.strftime('%d%b%Y_%H:%M:%S')}.npz"
    # np.savez(
    #     hist_fname,
    #     time=np.array(time_hist),
    #     desired_state=np.array(desired_state_hist),
    #     true_state=np.array(true_state_hist),
    #     desired_thrust_torque=np.array(desired_thrust_torque_hist),
    #     true_thrust_torque=np.array(true_thrust_torque_hist)
    #    )
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

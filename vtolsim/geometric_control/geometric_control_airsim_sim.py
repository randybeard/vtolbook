#/usr/bin/python3
"""
vtolsim
    - Update history:
        5/8/2019 - R.W. Beard
"""

import sys
import os
sys.path.append('..')
sys.path.append('../viewers')
sys.path.append('../trajectorygenerator/scripts')
import numpy as np
import time
import parameters.simulation_parameters_airsim as SIM
import parameters.convergence_parameters as VTOL

if SIM.viewer:
    from viewers.vtol_viewer import vtolViewer
if SIM.plots:
    from viewers.state_viewer import stateViewer
    from viewers.actuators_viewer import actuatorsViewer
    from viewers.thrust_torque_viewer import ThrustTorqueViewer
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

# AirSim
import airsim
from airsim.types import Pose, TiltrotorState, KinematicsState, RotorTiltableStates, Vector3r
import json

# trajectories
from trajectorygenerator.scripts.trajectory import Trajectory
from trajectorygenerator.scripts.sinusoidal_trajectory_member import SinusoidalTrajectoryMember
from trajectorygenerator.scripts.linear_trajectory_member import LinearTrajectoryMember
from trajectorygenerator.scripts.quadratic_trajectory_member import QuadraticTrajectoryMember
from trajectorygenerator.scripts import trajectory_plotter
from trajectory_planning.takeoff_coast_land_trajectory import TakeoffCoastLandTrajectory
from geometric_control.setup_polynomial_trajectory import setup_polynomial_acc_2021_tcl_trajectory
from geometric_control.setup_polynomial_trajectory import setup_polynomial_airsim_demo_tcl_trajectory
from geometric_control.setup_polynomial_trajectory import setup_bspline_airsim_demo_trajectory
from geometric_control.setup_polynomial_trajectory import setup_polynomial_airsim_rooftop_trajectory
from message_types.msg_controls import msgControls

np.set_printoptions(precision=4, linewidth=200, suppress=True)

def convert_delta_to_pwm(delta: msgControls):
    return [
        delta.elevon_left/VTOL.elevon_max,
        delta.elevon_right/VTOL.elevon_max,
        0.0,
        delta.throttle_left,
        -(2*delta.servo_left/VTOL.servo_max - 1.0), # flip left servo sign
        delta.throttle_right,
        2*delta.servo_right/VTOL.servo_max - 1.0,
        delta.throttle_rear,
        0.0
    ]

def convert_from_airsim_state(airsim_state: TiltrotorState, rotor_states: RotorTiltableStates):
    kinem: KinematicsState = airsim_state.kinematics_estimated
    rotors = rotor_states.rotors # a list of RotorTiltableParameters (C++ class)
    return np.array([
        kinem.position.x_val,
        kinem.position.y_val,
        kinem.position.z_val,
        kinem.linear_velocity.x_val,
        kinem.linear_velocity.y_val,
        kinem.linear_velocity.z_val,
        kinem.orientation.w_val,
        kinem.orientation.x_val,
        kinem.orientation.y_val,
        kinem.orientation.z_val,
        kinem.angular_velocity.x_val,
        kinem.angular_velocity.y_val,
        kinem.angular_velocity.z_val,
        rotors[1]['angle'] + VTOL.servo_max/2.0,
        -rotors[0]['angle'] + VTOL.servo_max/2.0,
    ]).reshape(15,1)

class GeometricControlAirSim:
    def __init__(self):
        # initialize AirSim client
        self.airsim_clockspeed = 1.0
        if SIM.use_airsim:
            self.client = airsim.TiltrotorClient()
            self.client.confirmConnection()
            s = self.client.getSettingsString()
            try:
                self.airsim_clockspeed = json.loads(s)['ClockSpeed']
            except KeyError:
                print('vtolsim: using default value of 1.0 for airsim clockspeed scaler')

            self.pwm_cmd_init = [
                0.,   # elevon left
                0.,   # elevon right
                0.,   # not used
                0.2,  # throttle left
               -0.5,  # tilt left
                0.2,  # throttle right
                0.5,  # tilt right
                0.3,  # throttle rear
                0.    # not used
            ]

        # initialize viewers
        if SIM.plots:
            self.state_view = stateViewer()
            self.actuators_view = actuatorsViewer()
            self.thrust_torque_view = ThrustTorqueViewer()
        if SIM.viewer:
            self.vtol_view = vtolViewer()

        # initialize elements of the architecture
        self.wind = windSimulation()
        self.vtol = vtolDynamics()

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

        poly_rooftop_trajectory = setup_polynomial_airsim_rooftop_trajectory(.10, optimize_segment_times=False)


        ## ---------------------------------
        #  Edit here to change the trajectory that gets used

        # self.sim_trajectory = circle_trajectory; traj_name = "Circle"; self.end_time = 30.
        # self.sim_trajectory = pringle_trajectory; traj_name = "Pringle"
        # self.sim_trajectory = line_trajectory; traj_name = "Line"
        # self.sim_trajectory = tcl_trajectory; traj_name = "TCL"; self.end_time = tcl_trajectory.t6
        # self.sim_trajectory = acc_poly_tcl_trajectory; traj_name = "ACCPolyTCL"; self.end_time = acc_poly_tcl_trajectory.members[0].num_segments/acc_poly_tcl_trajectory.time_scale + 0.5 #+ 5
        # self.sim_trajectory = airsim_poly_tcl_trajectory_opt; traj_name = "AirSimPolyTCLOpt"; self.end_time = airsim_poly_tcl_trajectory_opt.members[0].num_segments/airsim_poly_tcl_trajectory_opt.time_scale  + 5
        # self.sim_trajectory = airsim_poly_tcl_trajectory_nopt; traj_name = "AirSimPolyTCLNOpt"; self.end_time = airsim_poly_tcl_trajectory_nopt.members[0].num_segments/airsim_poly_tcl_trajectory_nopt.time_scale  + 5
        # self.sim_trajectory = airsim_bspline_tcl_trajectory; traj_name = "AirSimBSplineTCL"; self.end_time = airsim_bspline_tcl_trajectory.members[0].total_time/airsim_bspline_tcl_trajectory.time_scale  + 5
        self.sim_trajectory = poly_rooftop_trajectory; traj_name = "RooftopPoly"; self.end_time = poly_rooftop_trajectory.members[0].num_segments/poly_rooftop_trajectory.time_scale + 0.5

        ## ---------------------------------

        # draw the trajectory
        npts = int((self.end_time - SIM.start_time)/.01)
        trajectory_position_points, _ = trajectory_plotter.evalVector(self.sim_trajectory, 0, SIM.start_time, self.end_time, npts)

        if SIM.viewer:
            self.vtol_view.addTrajectory(trajectory_position_points[:3,:])

        # initialize geometric controller
        self.geom_ctrl = GeometricController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)
        # self.geom_ctrl = GeometricZThrustController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)

        #initialize low level control
        self.rate_control = RateControl(ts_control=SIM.ts_control)
        self.control_alloc = ControlAllocation(servo0=np.pi/2)

        # initialize command message
        self.delta = msgControls()

        #calculate_trim
        self.vtol._update_true_state()

        # initialize the simulation time
        self.sim_time = SIM.start_time
        self.Ts = SIM.ts_simulation
        self.dt_loop_target = SIM.dt_loop_target / self.airsim_clockspeed

        self.comp_time_hist = []

        self.perf_hist = []
        self.airsim_dt_hist = []

        self.init = False

    def unpause_airsim(self):
        if SIM.use_airsim and SIM.use_simContinueForTime:
            self.client.simPause(False)

    def close_viewers(self):
        if SIM.viewer:
            self.vtol_view.tick()
        if SIM.plots:
            self.state_view.tick()
            self.actuators_view.tick()
            self.thrust_torque_view.tick()

    def animate_ramp_up(self):
        tilt_duration = 0.5  # sec
        num_tilt_cmds = 50
        fracs = np.linspace(0.0, 1.0, num_tilt_cmds)
        for frac in fracs:
            tilt_cmd = self.pwm_cmd_init[6] * frac
            cmd_list = [0., 0., 0., 0., -tilt_cmd, 0., tilt_cmd, 0., 0.]
            self.client.moveByMotorPWMsAsync(cmd_list, tilt_duration/num_tilt_cmds).join()
        throttle_duration = 2.0  # sec
        num_thr_cmds = 50
        fracs = np.linspace(0.0, 1.0, num_thr_cmds)
        for frac in fracs:
            thr_cmd = self.pwm_cmd_init[3] * frac
            rear_thr_cmd = self.pwm_cmd_init[7] * frac
            tilt_cmd = self.pwm_cmd_init[6]
            cmd_list = [0., 0., 0., thr_cmd, -tilt_cmd, thr_cmd, tilt_cmd, rear_thr_cmd, 0.]
            self.client.moveByMotorPWMsAsync(cmd_list, throttle_duration/num_thr_cmds).join()
        self.client.moveByMotorPWMsAsync(self.pwm_cmd_init, 0.2)

    def run(self):
        # main simulation loop
        if SIM.use_airsim:
            self.animate_ramp_up()

            airsim_time_start = self.client.getTiltrotorState().timestamp
            airsim_time_prev = airsim_time_start
            airsim_dt_ns = 0.01 * 1.0e9 # initial guesstimate; will be computed each iteration

        # can use self.end_time to stop at end of trajectory or SIM.end_time
        print(f"\nBegin loop...")
        print(f"self.end_time: {self.end_time}")
        if self.airsim_clockspeed != 1.0:
            print(f"airsim ClockSpeed: {self.airsim_clockspeed}")
        self.loop_timer_start = time.perf_counter()
        dt_loop_actual = SIM.dt_loop_target
        t_start = time.perf_counter()  # begin recording total wall time elapsed for sim
        while self.sim_time < self.end_time:
            t1 = time.perf_counter()
            dt_run = t1 - self.loop_timer_start  # stop recording running time of loop

            #------- sleep logic to control main loop rate --------
            if not self.init:
                self.init = True
            elif dt_run < SIM.dt_loop_target:
                time.sleep(SIM.dt_loop_target - dt_run)
                dt_loop_actual = (time.perf_counter() - self.loop_timer_start) * self.airsim_clockspeed
            else:
                # loop took longer to execute than target time, so don't sleep
                dt_loop_actual = dt_run * self.airsim_clockspeed

            self.loop_timer_start = time.perf_counter()  # begin recording running time of loop
            self.Ts = dt_loop_actual
            #-------observer-------------
            estimated_state = self.vtol._state  # estimated state is current state

            ctrl_start_time = time.time()
            # ------ Trajectory follower
            traj_derivatives_at_t = self.sim_trajectory.evalUpToKr(self.sim_time, 4)

            #------- High Level controller-------------
            T, R_d, omega_c, pd_i, vd_b = self.geom_ctrl.update(estimated_state[0:10], traj_derivatives_at_t, self.Ts)

            #------- Low Level Controller -------------
            omega = estimated_state[10:13,0]
            tau_c = self.rate_control.update(omega_c, omega, self.Ts)
            self.delta = self.control_alloc.update(T, tau_c, self.vtol._Va)
            pwm_cmds = convert_delta_to_pwm(self.delta)

            ctrl_end_time = time.time()

            if SIM.use_airsim:
                self.client.moveByMotorPWMsAsync(pwm_cmds, duration=0.3)
                airsim_state = self.client.getTiltrotorState()
                rotor_states = self.client.getRotorStates()

                airsim_dt_ns = airsim_state.timestamp - airsim_time_prev
                airsim_time_prev = airsim_state.timestamp

                if SIM.use_simContinueForTime:
                    self.client.simContinueForTime(self.Ts)
                    self.unpause_airsim()

                self.vtol.external_set_state(convert_from_airsim_state(airsim_state, rotor_states))

            #-------update viewers-------------

            if SIM.viewer:
                self.vtol_view.update(self.vtol.true_state)
            if SIM.plots:
                #-------update physical system-------------
                # current_wind = self.wind.update()  # get the new wind vector
                self.vtol.update(self.delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics
                # self.vtol.update(self.delta, current_wind)  # propagate the MAV dynamics

                desired_state = np.concatenate([
                    pd_i,
                    vd_b,
                    Rotation2Quaternion(R_d).reshape(-1),
                    omega_c]).reshape((-1, 1))
                thrust_torque_d = np.concatenate([T, tau_c]).reshape((-1,1))
                thrust_torque = np.concatenate([np.copy(self.vtol.total_thrust[[0,2]]), np.copy(self.vtol.total_torque)])

                self.state_view.update(self.vtol._state, estimated_state, desired_state, omega_c, self.Ts)
                self.actuators_view.update(self.delta, self.vtol._state[13:15], self.Ts)
                self.thrust_torque_view.update(thrust_torque.reshape(-1,1), thrust_torque_d.reshape(-1,1), self.Ts)

            self.comp_time_hist.append(ctrl_end_time - ctrl_start_time)

            #-------increment time-------------
            self.sim_time += self.Ts

            #------- append to time histories --------
            self.perf_hist.append(self.Ts / self.airsim_clockspeed)
            if SIM.use_airsim:
                self.airsim_dt_hist.append(airsim_dt_ns)

        t_end = time.perf_counter()
        if SIM.use_airsim:
            airsim_time = self.client.getTiltrotorState().timestamp

        print("\n --- Done with simulation ---\n")

        t = t_end - t_start
        start_index = 20  # discard first few runs
        perf_hist_trim = self.perf_hist[start_index:]

        loop_dt_avg = np.mean(perf_hist_trim)
        loop_dt_min = np.min(perf_hist_trim)
        loop_dt_max = np.max(perf_hist_trim)

        if SIM.use_airsim:
            total_airsim_time = (airsim_time - airsim_time_start) * 1e-9
            print(f"Total vtolsim time (sim_time):   {self.sim_time:0.4f}")
            print(f"Total airsim time (end - start): {total_airsim_time:0.4f}")

            self.unpause_airsim()

        msg = "" if self.airsim_clockspeed == 1.0 else "(relative to airsim's clock) "
        print(f"\n --- vtolsim performance {msg}---\n")
        print(f"Total wall time: {t:8.4f} s")
        print(f"   Avg loop rate: {1.0/loop_dt_avg:8.4f} Hz, {loop_dt_avg*1e3:8.4f} ms")
        print(f"   Max loop rate: {1.0/loop_dt_min:8.4f} Hz, {loop_dt_min*1e3:8.4f} ms, index: {np.argmin(perf_hist_trim)}")
        print(f"   Min loop rate: {1.0/loop_dt_max:8.4f} Hz, {loop_dt_max*1e3:8.4f} ms, index: {np.argmax(perf_hist_trim)}")
        # print("Min computation rate = ", 1./np.max(self.comp_time_hist), " idx = ", np.argmax(self.comp_time_hist))
        # print("Avg computation rate = ", 1./np.mean(self.comp_time_hist))

        self.close_viewers()

if __name__ == '__main__':
    sim = GeometricControlAirSim()
    try:
        sim.run()
    except KeyboardInterrupt:
        try:
            sim.close_viewers()
            sim.unpause_airsim()
            sys.exit(0)
        except SystemExit:
            os._exit(0)

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
from control_allocation.nonlinear_control_allocation import NonlinearControlAllocation
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
from polynomial_trajectory_generator import PolynomialTrajectoryGenerator as PTG
from basis.standard_basis_member import StandardBasisMember as SBM
import utils.math_tools as mt

#argparser
import argparse

def main():

    def formatted_array(a):
        output = '['
        for i in range(0, len(a)):
            output += str(a[i])
            if i != (len(a) - 1):
                output += ', '
            else:
                output += ']'
        return output

    def create_log(log_file_arg, first_line):
        if log_file_arg is not None:
            with open(log_file_arg[0], 'w') as f:
                print(first_line, file=f, end='\n  ')
                np.set_printoptions(precision=4)
        
    def write_log_info(log_file, log_info, first_iter):
        with open(log_file, 'a') as f:
            if not first_iter:
                print('\n  ', file=f, end='')
            print('{}:'.format(round(sim_time, 2)), file=f)
            print(log_info, file=f, end='')

    def write_state_log(log_file, state, first_iter):
        log_info =  '    position: {}\n'.format(formatted_array(state[0:3][:,0]))
        log_info += '    velocity: {}\n'.format(formatted_array(state[3:6][:,0]))
        log_info += '    attitude: {}\n'.format(formatted_array(state[6:10][:,0]))  
        log_info += '    angular_rate: {}'.format(formatted_array(state[10:13][:,0]))
        write_log_info(log_file, log_info, first_iter)    

    def write_traj_log(log_file, traj_derivatives, first_iter):
        log_info =  '    position: {}\n'.format(formatted_array(traj_derivatives[0:3,0]))
        log_info += '    velocity: {}\n'.format(formatted_array(traj_derivatives[0:3,1]))
        log_info += '    acceleration: {}\n'.format(formatted_array(traj_derivatives[0:3,2]))
        log_info += '    jerk: {}\n'.format(formatted_array(traj_derivatives[0:3,3]))
        log_info += '    yaw: {}\n'.format(traj_derivatives[3,0])
        log_info += '    yaw_derivative: {}'.format(traj_derivatives[3,1])
        write_log_info(log_file, log_info, first_iter)

    def write_pitch_thrust_log(log_file, R_pi, omega_pip, T_db, first_iter):
        log_info =  '    R_pi: {}\n'.format(formatted_array(Rotation2Quaternion(R_pi)[0:4,0]))
        log_info += '    omega_pip: {}\n'.format(formatted_array(omega_pip))
        log_info += '    T_db: {}'.format(formatted_array(T_db))
        write_log_info(log_file, log_info, first_iter)

    def write_attitude_control_log(log_file, omega_c, first_iter):
        log_info = '    omega_c: {}'.format(formatted_array(omega_c))
        write_log_info(log_file, log_info, first_iter)

    def write_rate_ctrl_log(log_file, thrust, torque, first_iter):
        log_info =  '    thrust: {}\n'.format(formatted_array(thrust))
        log_info += '    torque: {}'.format(formatted_array(torque))
        write_log_info(log_file, log_info, first_iter)

    def write_ctrl_alloc_log(log_file, actuator_setpoints, first_iter):
        log_info =  '    throttle_right: {}\n'.format(actuator_setpoints.throttle_right)
        log_info += '    throttle_left: {}\n'.format(actuator_setpoints.throttle_left)
        log_info += '    throttle_rear: {}\n'.format(actuator_setpoints.throttle_rear)
        log_info += '    servo_right: {}\n'.format(actuator_setpoints.servo_right)
        log_info += '    servo_left: {}\n'.format(actuator_setpoints.servo_left)
        log_info += '    elevon_right: {}\n'.format(actuator_setpoints.elevon_right)
        log_info += '    elevon_left: {}'.format(actuator_setpoints.elevon_left)
        write_log_info(log_file, log_info, first_iter)

            
    # Initialize logging
    parser = argparse.ArgumentParser()
    parser.add_argument('--state_log', nargs=1, 
        help='log file containing the estimated state at each time-step')
    parser.add_argument('--traj_log', nargs=1, help='log file containing the trajectory at each time-step')
    parser.add_argument('--pitch_thrust_log', nargs=1, 
        help='log file containing the pitch and thrust allocation values at each time-step')
    parser.add_argument('--attitude_control_log', nargs=1, 
        help='log file containing the output rotation rates from the attitude control at each time-step')
    parser.add_argument('--rate_control_log', nargs=1,
        help='log file containing the output thrusts and torques from the rate control at each time-step')
    parser.add_argument('--ctrl_alloc_log', nargs=1,
        help='log file containing actuator outputs at each time-step')
    parser.add_argument('--nonlinear_alloc', action='store_true')
    args = parser.parse_args()

    first_iter = True
    create_log(args.state_log, 'state_log:')
    create_log(args.traj_log, 'traj_log:')
    create_log(args.pitch_thrust_log, 'pitch_thrust_log:')
    create_log(args.attitude_control_log, 'angular_rates_log:')
    create_log(args.rate_control_log, 'rate_control_log:')
    create_log(args.ctrl_alloc_log, 'ctrl_alloc_log:')
    USE_NONLINEAR_ALLOC = args.nonlinear_alloc

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

    # Commented this all out belows
    # acc_poly_tcl_trajectory = setup_polynomial_acc_2021_tcl_trajectory(.2, optimize_segment_times=False)
    # airsim_poly_tcl_trajectory_nopt = setup_polynomial_airsim_demo_tcl_trajectory(.2, optimize_segment_times=False)
    # airsim_poly_tcl_trajectory_opt = setup_polynomial_airsim_demo_tcl_trajectory(.2, optimize_segment_times=True)
    # airsim_bspline_tcl_trajectory = setup_bspline_airsim_demo_trajectory(.4)
    # bspline_time_scale = airsim_bspline_tcl_trajectory.members[0].total_time*.2/airsim_poly_tcl_trajectory_opt.members[0].total_time
    # airsim_bspline_tcl_trajectory.time_scale = bspline_time_scale


    ## ---------------------------------
    #  Edit here to change the trajectory that gets used

    # sim_trajectory = circle_trajectory; traj_name = "Circle"; SIM.end_time = 30.
    # sim_trajectory = pringle_trajectory; traj_name = "Pringle"
    # sim_trajectory = line_trajectory; traj_name = "Line"
    # sim_trajectory = tcl_trajectory; traj_name = "TCL"; SIM.end_time = tcl_trajectory.t6
    # sim_trajectory = acc_poly_tcl_trajectory; traj_name = "ACCPolyTCL"; SIM.end_time = acc_poly_tcl_trajectory.members[0].num_segments/acc_poly_tcl_trajectory.time_scale  + 5
    
    def setup_my_trajectory():
        waypoints = \
                [[0.0, 0.0, -0.0],
                [1.5, 0.0, -0.1],
                [4.5, 0.0, -0.3],
                [9.0, 0.0, -0.6],
                [15.0, 0.0, -1.0],
                [22.5, 0.0, -1.5],
                [31.5, 0.0, -2.0],
                [42.0, 0.0, -2.5],
                [54.0, 0.0, -3.0],
                [67.5, 0.0, -3.5],
                [82.5, 0.0, -3.9],
                [99.0, 0.0, -4.2],
                [117.0, 0.0, -4.4],
                [136.5, 0.0, -4.5],
                [157.5, 0.0, -4.5],
                [180.0, 0.0, -4.5],
                [204.0, 0.0, -4.5],
                [229.5, 0.0, -4.5],
                [255.0, 0.0, -4.5],
                [280.5, 0.0, -4.5],
                [306.0, 0.0, -4.5],
                [331.5, 0.0, -4.5],
                [357.0, 0.0, -4.5],
                [382.5, 0.0, -4.5],
                [408.0, 0.0, -4.5],
                [433.5, 0.0, -4.5],
                [459.0, 0.0, -4.5],
                [484.5, 0.0, -4.5],
                [509.4, 0.0, -4.5],
                [533.7, 0.0, -4.5],
                [557.4, 0.0, -4.5],
                [580.5, 0.0, -4.5],
                [603.0, 0.0, -4.5],
                [624.9, 0.0, -4.5],
                [646.2, 0.0, -4.5],
                [666.9, 0.0, -4.5],
                [687.0, 0.0, -4.5],
                [706.5, 0.0, -4.5],
                [725.4, 0.0, -4.5],
                [743.7, 0.0, -4.5],
                [761.4, 0.0, -4.5],
                [778.5, 0.0, -4.5],
                [795.0, 0.0, -4.5],
                [810.9, 0.0, -4.5],
                [826.2, 0.0, -4.5],
                [840.9, 0.0, -4.5],
                [855.0, 0.0, -4.5],
                [868.5, 0.0, -4.5],
                [881.4, 0.0, -4.5],
                [893.7, 0.0, -4.5],
                [905.4, 0.0, -4.5],
                [916.5, 0.0, -4.5],
                [927.0, 0.0, -4.5],
                [936.9, 0.0, -4.5],
                [946.2, 0.0, -4.5],
                [954.9, 0.0, -4.5],
                [963.0, 0.0, -4.5],
                [970.5, 0.0, -4.5],
                [977.4, 0.0, -4.5],
                [983.7, 0.0, -4.5],
                [989.4, 0.0, -4.4],
                [994.5, 0.0, -4.2],
                [999.0, 0.0, -3.9],
                [1002.9, 0.0, -3.5],
                [1006.2, 0.0, -3.0],
                [1008.9, 0.0, -2.5],
                [1011.0, 0.0, -2.0],
                [1012.5, 0.0, -1.5],
                [1013.4, 0.0, -1.0],
                [1013.7, 0.0, -0.6],
                [1013.7, 0.0, -0.3],
                [1013.7, 0.0, -0.1],
                [1013.7, 0.0, -0.0]]
        # Path used for graphing:
        # waypoints = \
        #     [[0.0, 0.0, 0.0],
        #     [1.0, 0.0, -0.5],
        #     [2.0, 0.0, -1.0],
        #     [3.0, 0.0, -2.5],
        #     [4.0, 0.0, -4.0],
        #     [5.0, 0.0, -6.0],
        #     [6.5, 0.0, -6.0],
        #     [8.5, 0.0, -6.0],
        #     [11.0, 0.0, -6.0],
        #     [14.0, 0.0, -6.0],
        #     [17.5, 0.0, -6.0],
        #     [21.5, 0.0, -6.0],
        #     [26.0, 0.0, -6.0],
        #     [31.0, 0.0, -6.0],
        #     [36.5, 0.0, -6.0],
        #     [42.5, 0.0, -6.0],
        #     [49.0, 0.0, -6.0],
        #     [56.0, 0.0, -6.0],
        #     [63.5, 0.0, -6.0],
        #     [71.5, 0.0, -6.0],
        #     [80.0, 0.0, -6.0],
        #     [89.0, 0.0, -6.0],
        #     [98.5, 0.0, -6.0],
        #     [108.5, 0.0, -6.0],
        #     [119.0, 0.0, -6.0],
        #     [130.0, 0.0, -6.0],
        #     [141.5, 0.0, -6.0],
        #     [153.5, 0.0, -6.0],
        #     [166.0, 0.0, -6.0],
        #     [179.0, 0.0, -6.0],
        #     [192.5, 0.0, -6.0],
        #     [206.5, 0.0, -6.0],
        #     [221.0, 0.0, -6.0],
        #     [236.0, 0.0, -6.0],
        #     [251.5, 0.0, -6.0],
        #     [267.5, 0.0, -6.0],
        #     [284.0, 0.0, -6.0],
        #     [301.0, 0.0, -6.0],
        #     [318.5, 0.0, -6.0],
        #     [336.5, 0.0, -6.0],
        #     [355.0, 0.0, -6.0],
        #     [374.0, 0.0, -6.0],
        #     [393.5, 0.0, -6.0],
        #     [413.5, 0.0, -6.0],
        #     [434.0, 0.0, -6.0],
        #     [455.0, 0.0, -6.0],
        #     [476.5, 0.0, -6.0],
        #     [498.5, 0.0, -6.0],
        #     [521.0, 0.0, -6.0],
        #     [544.0, 0.0, -6.0]]

        # Set up position trajectory
        num_waypoints = len(waypoints)
        waypoints = np.array(waypoints)
        if waypoints.shape[0] != num_waypoints:
            print("Invalid number of waypoints")
            return
        pos_basis_degree = 9
        basis_class = SBM
        pos_continuity_derivative = 4
        pos_smoothness_derivative = 4
        pos_knot_con_structure_0_d = [{'deriv': 0, 'knot_list': 'all'}]
        pos_knot_con_structure_high_d = []
        pos_knot_con_structure_high_d.append({'deriv': 1, 'knot_list': [0]})
        pos_knot_con_structure = pos_knot_con_structure_0_d + pos_knot_con_structure_high_d

        pos_traj_generator = PTG(num_waypoints - 1,
                        pos_basis_degree + 1,
                        basis_class,
                        pos_continuity_derivative,
                        pos_smoothness_derivative,
                        pos_knot_con_structure)

        # Set up yaw trajectory
        yaw_basis_degree = 9
        yaw_continuity_derivative = 4
        yaw_smoothness_derivative = 4
        yaw_knot_con_structure_high_d = []
        yaw_knot_con_structure_high_d.append({'deriv': 1, 'knot_list': [0]})
        yaw_knot_con_structure_0_d = [{'deriv': 0, 'knot_list': 'all'}]
        yaw_knot_con_structure = yaw_knot_con_structure_0_d + yaw_knot_con_structure_high_d

        yaw_traj_generator = PTG(num_waypoints - 1,
                        yaw_basis_degree + 1,
                        basis_class,
                        yaw_continuity_derivative,
                        yaw_smoothness_derivative,
                        yaw_knot_con_structure)

        # Position trajectory
        current_velocity = np.zeros(3)
        kcv_x_0_d = [{'deriv': 0, 'value_list': waypoints[:, 0]}]
        kcv_x_high_d = []
        kcv_x_high_d.append({'deriv': 1, 'value_list': [current_velocity[0]]})
        kcv_x = kcv_x_0_d + kcv_x_high_d
        kcv_y_0_d = [{'deriv': 0, 'value_list': waypoints[:, 1]}]
        kcv_y_high_d = [{'deriv': 1, 'value_list': [current_velocity[1]]}]
        kcv_y = kcv_y_0_d + kcv_y_high_d
        kcv_z_0_d = [{'deriv': 0, 'value_list': waypoints[:, 2]}]
        kcv_z_high_d = [{'deriv': 1, 'value_list': [current_velocity[2]]}]
        kcv_z = kcv_z_0_d + kcv_z_high_d
        pos_trajectory = pos_traj_generator.generate_trajectory([kcv_x, kcv_y, kcv_z])

        # Yaw trajectory
        current_yaw = 0
        time_scale = 1.0
        pos_trajectory.time_scale = time_scale
        
        vel_list = []
        for i in range(num_waypoints):
            vel_list.append(pos_trajectory.eval(i/time_scale, 1))
        yaw_points = mt.velocities_to_headings(vel_list, current_yaw)
        kcv_yaw_high_d = [{'deriv': 1, 'value_list': [0]}]
        kcv_yaw_0_d = [{'deriv': 0, 'value_list': yaw_points}]
        kcv_yaw = kcv_yaw_0_d + kcv_yaw_high_d
        yaw_trajectory = (yaw_traj_generator.generate_trajectory([kcv_yaw]))
        yaw_trajectory.time_scale = time_scale

        return pos_trajectory, yaw_trajectory
    pos_traj, yaw_traj = setup_my_trajectory()
    traj_name = 'MyTrajectory'
    # SIM.end_time = pos_trajectory.members[0].total_time/pos_trajectory.time_scale  + 5

    # sim_trajectory = airsim_poly_tcl_trajectory_opt; traj_name = "AirSimPolyTCLOpt"; SIM.end_time = airsim_poly_tcl_trajectory_opt.members[0].num_segments/airsim_poly_tcl_trajectory_opt.time_scale  + 5
    # sim_trajectory = airsim_poly_tcl_trajectory_nopt; traj_name = "AirSimPolyTCLNOpt"; SIM.end_time = airsim_poly_tcl_trajectory_nopt.members[0].num_segments/airsim_poly_tcl_trajectory_nopt.time_scale  + 5
    # sim_trajectory = airsim_bspline_tcl_trajectory; traj_name = "AirSimBSplineTCL"; SIM.end_time = airsim_bspline_tcl_trajectory.members[0].total_time/airsim_bspline_tcl_trajectory.time_scale  + 5

    ## ---------------------------------

    # draw the trajectory
    npts = int((SIM.end_time - SIM.start_time)/.01)
    trajectory_position_points, _ = trajectory_plotter.evalVector(pos_traj, 0, SIM.start_time, SIM.end_time, npts)

    vtol_view.addTrajectory(trajectory_position_points[:3,:])

    # initialize geometric controller
    geom_ctrl = GeometricController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)
    # geom_ctrl = GeometricZThrustController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)

    #initialize low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    control_alloc = NonlinearControlAllocation()


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
        if args.state_log is not None:
            write_state_log(args.state_log[0], estimated_state, first_iter)

        ctrl_start_time = time.time()
        # ------ Trajectory follower
        pos_derivatives_at_t = pos_traj.evalUpToKr(sim_time, 4)
        yaw_derivatives_at_t = yaw_traj.evalUpToKr(sim_time, 4)
        # print('traj_derivatives_at_t')
        # print(pos_derivatives_at_t)
        # print(yaw_derivatives_at_t)
        # print(np.concatenate((pos_derivatives_at_t, yaw_derivatives_at_t), axis=0))
        traj_derivatives_at_t = np.concatenate((pos_derivatives_at_t, yaw_derivatives_at_t), axis=0)
        # traj_derivatives_at_t = np.zeros(16).reshape((4,4))
        if args.traj_log is not None:
            write_traj_log(args.traj_log[0], traj_derivatives_at_t, first_iter)

        #------- High Level controller-------------
        T, R_d, omega_c, pd_i, vd_b, omega_pip = geom_ctrl.update(estimated_state[0:10], traj_derivatives_at_t)
        if args.pitch_thrust_log is not None:
            write_pitch_thrust_log(args.pitch_thrust_log[0], R_d, omega_pip, T, first_iter)
        if args.attitude_control_log is not None:
            write_attitude_control_log(args.attitude_control_log[0], omega_c, first_iter)

        #------- Low Level Controller -------------
        omega = estimated_state[10:13,0]
        tau_c = rate_control.update(omega_c, omega)
        if args.rate_control_log is not None:
            write_rate_ctrl_log(args.rate_control_log[0], T, tau_c, first_iter)
        # print(round(sim_time, 2))
        # delta = control_alloc.update(T, tau_c, estimated_state, vtol._Va)
        delta = control_alloc.update(T, tau_c, estimated_state, vtol._Va)
        # delta = control_alloc.update_lp(T, tau_c, vtol._Va)
        # delta = control_alloc.update_combined_methods(T, tau_c, vtol._Va)
        if USE_NONLINEAR_ALLOC:
            delta = control_alloc.update(T, tau_c, estimated_state, vtol._Va)
        else:
            delta = control_alloc.update(T, tau_c, vtol._Va)
        if args.ctrl_alloc_log is not None:
            write_ctrl_alloc_log(args.ctrl_alloc_log[0], delta, first_iter)
        ctrl_end_time = time.time()

        #-------update physical system-------------
        current_wind = wind.update()  # get the new wind vector
        ttd = np.array([[T[0], 0, T[1], tau_c[0], tau_c[1], tau_c[2]]])
        vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T, ttd.T)  # propagate the MAV dynamics
        # vtol.update(delta, current_wind)  # propagate the MAV dynamics

        #-------update viewers-------------
        desired_state = np.concatenate([
            pd_i,
            vd_b,
            Rotation2Quaternion(R_d).reshape(-1),
            omega_c]).reshape((-1, 1))

        thrust_torque_d = np.concatenate([T, tau_c]).reshape((-1,1))
        # thrust_torque = np.concatenate([T, tau_c]).reshape((-1,1))
        thrust_torque = np.concatenate([np.copy(vtol.total_thrust[[0,2]]), np.copy(vtol.total_torque)])

        # vtol_view.update(vtol.true_state)
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
        first_iter = False
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

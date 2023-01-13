#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import yaml

from trajectory import Trajectory
from polynomial_trajectory_generator \
    import PolynomialTrajectoryGenerator as PTG
from basis.standard_basis_member import StandardBasisMember as SBM
import trajectory_plotter
import utils.math_tools as mt


def main():
    # for testing
    current_pos = [0., 0., 0.]
    current_vel = [0., 0., 0.]  # inertial frame
    current_yaw = 0.0
    current_yaw_rate = 0.0

    # load yaml file
    spline_traj_params_top = yaml.safe_load(
        open('../params/spline_traj.yaml', 'r'))
    traj_params = spline_traj_params_top['trajectory_generator']

    # create Polynomial Trajectory Generator
    """Setup position and yaw trajectory generators"""
    basis_type = traj_params['basis_type']
    pos_basis_degree = traj_params['pos_basis_degree']

    if basis_type == 'standard':
        basis_class = SBM
    else:
        print("[Trajectory Node] - Basis type not recognized!")
        raise ValueError

    num_waypoints = traj_params['num_waypoints']
    prepend_current_state = traj_params['prepend_current_state']
    if prepend_current_state:
        num_waypoints += 1

    pos_continuity_derivative = traj_params['pos_continuity_derivative']
    yaw_continuity_derivative = traj_params['yaw_continuity_derivative']

    pos_smoothness_derivative = traj_params['pos_smoothness_derivative']
    yaw_smoothness_derivative = traj_params['yaw_smoothness_derivative']

    pos_knot_con_structure_high_d = \
        traj_params['pos_knot_con_structure_high_d']

    # Always constrain all waypoints to be at knots
    pos_knot_con_structure_0_d = [{'deriv': 0, 'knot_list': 'all'}]

    if prepend_current_state:
        pos_knot_con_structure_high_d = \
            modify_high_d_structure_for_prepend_current(
                pos_knot_con_structure_high_d)

    # Append desired structure for higher derivatives
    pos_knot_con_structure = \
        pos_knot_con_structure_0_d + pos_knot_con_structure_high_d

    # Create splines
    pos_traj_generator = PTG(
        num_waypoints - 1,
        pos_basis_degree + 1,
        basis_class,
        pos_continuity_derivative,
        pos_smoothness_derivative,
        pos_knot_con_structure)

    waypoints = traj_params['static_waypoints']
    if waypoints is not None:
        if prepend_current_state:
            waypoints = [current_pos] + waypoints
        waypoints = np.array(waypoints)
    else:
        print("No static waypoints in YAML file")
        exit()

    kcv_x_high_d = traj_params['pos_x_con_value_high_d']
    kcv_y_high_d = traj_params['pos_y_con_value_high_d']
    kcv_z_high_d = traj_params['pos_z_con_value_high_d']

    if prepend_current_state:
        kcv_x_high_d = modify_high_d_value_for_prepend_current(
            kcv_x_high_d, current_vel[0])
        kcv_y_high_d = modify_high_d_value_for_prepend_current(
            kcv_y_high_d, current_vel[1])
        kcv_z_high_d = modify_high_d_value_for_prepend_current(
            kcv_z_high_d, current_vel[2])

    kcv_x_0_d = [{'deriv': 0, 'value_list': waypoints[:, 0]}]
    kcv_x = kcv_x_0_d + kcv_x_high_d

    kcv_y_0_d = [{'deriv': 0, 'value_list': waypoints[:, 1]}]
    kcv_y = kcv_y_0_d + kcv_y_high_d

    kcv_z_0_d = [{'deriv': 0, 'value_list': waypoints[:, 2]}]
    kcv_z = kcv_z_0_d + kcv_z_high_d

    pos_trajectory = \
        pos_traj_generator.generate_trajectory([kcv_x, kcv_y, kcv_z])
    pos_trajectory_opt = \
        pos_traj_generator.generate_trajectory([kcv_x, kcv_y, kcv_z], optimize_segment_times=True)
    time_scale = traj_params['time_scale']
    pos_trajectory.time_scale = time_scale
    pos_trajectory_opt.time_scale = time_scale

    # yaw
    yaw_knot_con_structure_high_d = \
        traj_params['yaw_knot_con_structure_high_d']

    if prepend_current_state:
        yaw_knot_con_structure_high_d = \
            modify_high_d_structure_for_prepend_current(
                yaw_knot_con_structure_high_d)

    yaw_knot_con_structure_0_d = [{'deriv': 0, 'knot_list': 'all'}]
    yaw_knot_con_structure = \
        yaw_knot_con_structure_0_d + yaw_knot_con_structure_high_d

    yaw_traj_generator = PTG(
        num_waypoints - 1,  # number of segments
        traj_params['yaw_basis_degree'] + 1,
        basis_class,
        yaw_continuity_derivative,
        yaw_smoothness_derivative,
        yaw_knot_con_structure)

    vel_list = []
    for i in range(num_waypoints):
        vel_list.append(pos_trajectory.eval(i/time_scale, 1))

    # yaw_points = np.zeros(num_waypoints)
    # import pdb; pdb.set_trace()
    yaw_points = mt.velocities_to_headings(vel_list, current_yaw)
    # yaw_points[0] = current_yaw

    kcv_yaw_high_d = traj_params['yaw_con_value_high_d']
    if prepend_current_state:
        kcv_yaw_high_d = modify_high_d_value_for_prepend_current(
            kcv_yaw_high_d, current_yaw_rate)

    kcv_yaw_0_d = [{'deriv': 0, 'value_list': yaw_points}]
    kcv_yaw = kcv_yaw_0_d + kcv_yaw_high_d
    yaw_trajectory = \
        yaw_traj_generator.generate_trajectory([kcv_yaw])
    yaw_trajectory.time_scale = time_scale

    end_time = pos_trajectory.members[0].total_time/time_scale
    figd0, axd0 = trajectory_plotter.plotVsTime(
        pos_trajectory, 0, 0, end_time)
    _, _ = trajectory_plotter.plotVsTime(
        pos_trajectory_opt, 0, 0, end_time, fig=figd0, ax=axd0)

    figd1, axd1 = trajectory_plotter.plotVsTime(
        pos_trajectory, 1, 0, end_time)
    _, _ = trajectory_plotter.plotVsTime(
        pos_trajectory_opt, 1, 0, end_time, fig=figd1, ax=axd1)

    figd2, axd2 = trajectory_plotter.plotVsTime(
        pos_trajectory, 2, 0, end_time)
    _, _ = trajectory_plotter.plotVsTime(
        pos_trajectory_opt, 2, 0, end_time, fig=figd2, ax=axd2)

    figd3, axd3 = trajectory_plotter.plotVsTime(
        pos_trajectory, 3, 0, end_time)
    _, _ = trajectory_plotter.plotVsTime(
        pos_trajectory_opt, 3, 0, end_time, fig=figd3, ax=axd3)

    trajectory_plotter.plotVsTime(
        yaw_trajectory, 0, 0, end_time)
    trajectory_plotter.plotVsTime(
        yaw_trajectory, 1, 0, end_time)
    trajectory_plotter.plotVsTime(
        yaw_trajectory, 2, 0, end_time)
    trajectory_plotter.plotVsTime(
        yaw_trajectory, 3, 0, end_time)

    trajectory_plotter.plot3D(
        pos_trajectory, 0, [0, 1, 2], 0, end_time)
    plt.show()


def modify_high_d_structure_for_prepend_current(knot_con_structure_high_d):
    found_vel = False
    for i in range(len(knot_con_structure_high_d)):
        ks_deriv_dict = knot_con_structure_high_d[i]
        deriv_kl = np.array(ks_deriv_dict['knot_list'])
        # shift all positive indices
        deriv_kl[deriv_kl >= 0] += 1
        if ks_deriv_dict['deriv'] == 1:
            deriv_kl = np.insert(deriv_kl, 0, 0)
            found_vel = True
        knot_con_structure_high_d[i]['knot_list'] = deriv_kl

    if found_vel is False:
        # no existing velocity constraints
        knot_con_structure_high_d.append({'deriv': 1, 'knot_list': [0]})

    return knot_con_structure_high_d


def modify_high_d_value_for_prepend_current(knot_con_value_high_d, vel_value):
    kv_sorted = sorted(knot_con_value_high_d, key=lambda k: k['deriv'])
    if len(kv_sorted) > 0 and kv_sorted[0]['deriv'] == 1:
        kv_sorted[0]['value_list'].insert(0, vel_value)
    else:
        # no velocity in knot_con_value_high_d
        kv_sorted.append({'deriv': 1, 'value_list': [vel_value]})
    return kv_sorted


if __name__ == '__main__':
    main()

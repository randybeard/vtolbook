import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('../trajectorygenerator/scripts')
from polynomial_trajectory_generator import PolynomialTrajectoryGenerator
from bspline_trajectory_generator import BSplineTrajectoryGenerator
from basis.standard_basis_member import StandardBasisMember
import trajectory_plotter

filetype = ".pdf"
# filetype = ".png"

def setup_polynomial_acc_2021_tcl_trajectory(time_scale, optimize_segment_times=False):
    num_segments = 5
    num_bases = 7

    continuity_derivative = 3
    smoothness_derivative = 3

    knot_constraint_structure = [
        {'deriv': 0,
            'knot_list': [x for x in range(num_segments+1)]},
        {'deriv': 1,
            'knot_list': [0, num_segments]},
        {'deriv': 2,
            'knot_list': [num_segments]},
    ]

    basis_class = StandardBasisMember

    gen = PolynomialTrajectoryGenerator(
        num_segments,
        num_bases,
        basis_class,
        continuity_derivative,
        smoothness_derivative,
        knot_constraint_structure
    )

    kcv_x = [
        # position - all knot points
        {'deriv': 0,
            'value_list': [0., 20., 50., 75., 90., 100]},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_y = [
        # position - all knot points
        {'deriv': 0,
            'value_list': [0., 0., 0., 10., 15., 20.]},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_z = [
        # position - all knot points
        {'deriv': 0,
            'value_list': [0., -5., -10., -10., -5., 0.]},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_psi = [
        # position - all knot points
        {'deriv': 0,
            'value_list': [0., 0., 0., np.pi/8, np.pi/4, np.pi/2]},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]


    kcv_list = [kcv_x, kcv_y, kcv_z, kcv_psi]
    polynomial_tcl_traj = gen.generate_trajectory(kcv_list, optimize_segment_times=optimize_segment_times)
    polynomial_tcl_traj.time_scale = time_scale

    return polynomial_tcl_traj

def setup_polynomial_airsim_demo_tcl_trajectory(time_scale, optimize_segment_times=False):
    num_segments = 5
    num_bases = 7

    continuity_derivative = 3
    smoothness_derivative = 3

    knot_constraint_structure = [
        {'deriv': 0,
            'knot_list': [x for x in range(num_segments+1)]},
        {'deriv': 1,
            'knot_list': [0, num_segments]},
        {'deriv': 2,
            'knot_list': [num_segments]},
    ]

    basis_class = StandardBasisMember

    gen = PolynomialTrajectoryGenerator(
        num_segments,
        num_bases,
        basis_class,
        continuity_derivative,
        smoothness_derivative,
        knot_constraint_structure
    )

    kcv_x = [
        # position - all knot points
        {'deriv': 0,
            'value_list': [0., 30., 50., 70., 80., 80.]},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_y = [
        # position - all knot points
        {'deriv': 0,
            'value_list': [0., 0., 0., -5., -20., -25.]},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_z = [
        # position - all knot points
        {'deriv': 0,
            'value_list': [0., -5., -10., -10., -5., 0.]},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_psi = [
        # position - all knot points
        {'deriv': 0,
            'value_list': [0., 0., 0., -np.pi/8, -np.pi/3, -np.pi/2]},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]
    kcv_list = [kcv_x, kcv_y, kcv_z, kcv_psi]
    polynomial_tcl_traj = gen.generate_trajectory(kcv_list, optimize_segment_times=optimize_segment_times)
    polynomial_tcl_traj.time_scale = time_scale

    return polynomial_tcl_traj

def setup_bspline_airsim_demo_trajectory(time_scale):
    degree = 4

    path_points_x = np.array([0., 30., 50., 70., 80., 80.])
    path_points_y = np.array([0., 0., 0., -5., -20., -25.])
    path_points_z = np.array([0., -5., -10., -10., -5., 0.])
    path_points_psi = np.array([0., 0., 0., -np.pi/8, -np.pi/3, -np.pi/2])

    path_points = np.row_stack([path_points_x, path_points_y, path_points_z, path_points_psi])

    bspline_generator = BSplineTrajectoryGenerator()
    bspline_traj = bspline_generator.generate_trajectory(path_points, degree)
    bspline_traj.time_scale = time_scale

    return bspline_traj

def setup_polynomial_airsim_rooftop_trajectory(time_scale, optimize_segment_times=False):
    num_segments = 5
    num_bases = 7

    continuity_derivative = 3
    smoothness_derivative = 3

    knot_constraint_structure = [
        {'deriv': 0,
            'knot_list': [x for x in range(num_segments+1)]},
        {'deriv': 1,
            'knot_list': [0, num_segments]},
        {'deriv': 2,
            'knot_list': [num_segments]},
    ]

    basis_class = StandardBasisMember

    gen = PolynomialTrajectoryGenerator(
        num_segments,
        num_bases,
        basis_class,
        continuity_derivative,
        smoothness_derivative,
        knot_constraint_structure
    )

    max_xyz = np.array([206.0, 18.0, -20])  # inertial frame, relative to start position/attitude

    # These will define the points of trajectory, as fractions of "pos_end" and "max_altitude"
    # Each point is spaced uniformly in time along trajectory
    x = np.array([0.00,  0.30,  0.60,  0.80,  0.98,  1.00]) * max_xyz[0]
    y = np.array([0.00,  0.00,  0.00,  0.50,  0.80,  1.00]) * max_xyz[1]
    z = np.array([0.00,  0.60,  1.00,  1.00,  0.60,  0.00]) * max_xyz[2]
    z[-1] = 1.3  # manually setting final altitude

    # desired yaw at each of above points
    yaw = [0., 0., 0., np.pi/16, np.pi/16, np.pi/8]

    kcv_x = [
        # position - all knot points
        {'deriv': 0,
            'value_list': x.tolist()},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_y = [
        # position - all knot points
        {'deriv': 0,
            'value_list': y.tolist()},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_z = [
        # position - all knot points
        {'deriv': 0,
            'value_list': z.tolist()},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]

    kcv_psi = [
        # position - all knot points
        {'deriv': 0,
            'value_list': yaw},
        # velocity - first and last knot points
        {'deriv': 1,
            'value_list': [0., 0.]},
        # acceleration - 10th knot point
        {'deriv': 2,
            'value_list': [0.]}
    ]


    kcv_list = [kcv_x, kcv_y, kcv_z, kcv_psi]
    polynomial_tcl_traj = gen.generate_trajectory(kcv_list, optimize_segment_times=optimize_segment_times)
    polynomial_tcl_traj.time_scale = time_scale

    return polynomial_tcl_traj

def plot_polynomial_trajectory(trajs, legends, fname=None, thesis_fmt=True):
    if thesis_fmt:
        figd0, axd0 = plt.subplots(4, 1, sharex=True, figsize=(5,3))
        plt.subplots_adjust(right=.75, hspace=0.1, left=.15, bottom=.15, top=.99)
        figd1, axd1 = plt.subplots(4, 1, sharex=True, figsize=(5,3))
        plt.subplots_adjust(right=.75, hspace=0.1, left=.15, bottom=.15, top=.99)
        figd2, axd2 = plt.subplots(4, 1, sharex=True, figsize=(5,3))
        plt.subplots_adjust(right=.75, hspace=0.1, left=.15, bottom=.15, top=.99)
    else:
        figd0, axd0 = plt.subplots(4, 1, sharex=True, figsize=(2.15,3))
        plt.subplots_adjust(right=.99, hspace=0.1, left=.23, bottom=.12, top=.99)
        figd1, axd1 = plt.subplots(4, 1, sharex=True, figsize=(2.15,3))
        plt.subplots_adjust(right=.99, hspace=0.1, left=.23, bottom=.12, top=.99)
        figd2, axd2 = plt.subplots(4, 1, sharex=True, figsize=(2.15,3))
        plt.subplots_adjust(right=.99, hspace=0.1, left=.23, bottom=.12, top=.99)

    labels = ["North (m)", "East (m)", "Down (m)", "Yaw (rad)"]

    for traj in trajs:
        traj_time = traj.members[0].total_time/traj.time_scale
        _, _ = trajectory_plotter.plotVsTime(traj, 0, 0, traj_time, fig=figd0, ax=axd0, labels=labels)
        _, _ = trajectory_plotter.plotVsTime(traj, 1, 0, traj_time, fig=figd1, ax=axd1, labels=labels)
        _, _ = trajectory_plotter.plotVsTime(traj, 2, 0, traj_time, fig=figd2, ax=axd2, labels=labels)


    if thesis_fmt:
        axd0[0].legend(legends, loc="upper right", bbox_to_anchor=(1.42,1.0), borderpad=.2, labelspacing=.3)
        axd1[0].legend(legends, loc="upper right", bbox_to_anchor=(1.42,1.0), borderpad=.2, labelspacing=.3)
        axd2[0].legend(legends, loc="upper right", bbox_to_anchor=(1.42,1.0), borderpad=.2, labelspacing=.3)
    else:
        axd0[0].legend(legends, loc="upper right", borderpad=.2, labelspacing=.3)
        axd1[0].legend(legends, loc="upper right", borderpad=.2, labelspacing=.3)
        axd2[0].legend(legends, loc="upper right", borderpad=.2, labelspacing=.3)

    if fname is not None:
        figd0.suptitle("")
        figd1.suptitle("")
        figd2.suptitle("")

        figd0.savefig(fname + "_d0"+filetype, dpi=600)
        figd1.savefig(fname + "_d1"+filetype, dpi=600)
        figd2.savefig(fname + "_d2"+filetype, dpi=600)


    # plot n, e, d in 3D
    trajectory_plotter.plot3D(traj, 0, [0, 1, 2], 0, traj_time)

    plt.show()

def plot_opt_nonopt_comparison(thesis_fmt=True):
    time_scale=.2
    nonopt_traj = setup_polynomial_airsim_demo_tcl_trajectory(time_scale)
    opt_traj = setup_polynomial_airsim_demo_tcl_trajectory(time_scale, optimize_segment_times=True)

    bspline_traj = setup_bspline_airsim_demo_trajectory(time_scale)
    bspline_time_scale = bspline_traj.members[0].total_time*time_scale/opt_traj.members[0].total_time
    print("Bspline scale = ", bspline_time_scale)
    bspline_traj.time_scale = bspline_time_scale

    plot_polynomial_trajectory([nonopt_traj, opt_traj, bspline_traj], legends=["Uniform", "Optimized", "B-spline"], fname="traj_plots/airsim_demo_tcl", thesis_fmt=True)


if __name__ == "__main__":
    from matplotlib import rc

    thesis_fmt = True

    if thesis_fmt:
        rc('text', usetex=True)
        rc('text.latex', preamble=r'\usepackage{amsmath}')
        rc('font', size=10)
        rc('lines', linewidth=1)
        rc('mathtext', fontset='cm')
        rc('font', family='STIXGeneral')
        plot_opt_nonopt_comparison(thesis_fmt=True)
    else:
        rc('text', usetex=True)
        rc('text.latex', preamble=r'\usepackage{amsmath}')
        rc('font', size=8)
        rc('figure', figsize=(2.15,3))
        rc('lines', linewidth=.8)
        rc('mathtext', fontset='cm')
        rc('font', family='STIXGeneral')
        plot_opt_nonopt_comparison(thesis_fmt=False)

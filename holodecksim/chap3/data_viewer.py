"""
data_viewer

    - Update history:
        3/18/2020 - RWB
        6/23/2020 - RWB
"""
from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *
from tools.rotations import rotation_to_euler


class dataViewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=100,  # refresh plot every 100 time steps
                               time_window=time_window_length)  # plot last time_window seconds of data
        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn', 'pn_e', 'pn_c'],
                               labels={'left': 'north(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        pe_plots = PlotboxArgs(plots=['pe', 'pe_e', 'pe_c'],
                               labels={'left': 'east(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        h_plots = PlotboxArgs(plots=['h', 'h_e', 'h_c'],
                              labels={'left': 'altitude(m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        thrust_plot = PlotboxArgs(plots=['force'],
                                   labels={'left': 'force(N)', 'bottom': 'Time (s)'},
                                   time_window=time_window_length)
        first_row = [pn_plots, pe_plots, h_plots, thrust_plot]

        # define second row
        vx_plots = PlotboxArgs(plots=['velx', 'velx_e', 'velx_c'],
                               labels={'left': 'velocity_x(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        vy_plots = PlotboxArgs(plots=['vely', 'vely_e', 'vely_c'],
                               labels={'left': 'velocity_y(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        vz_plots = PlotboxArgs(plots=['velz', 'velz_e', 'velz_c'],
                               labels={'left': 'velocity_z(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        torque_plot = PlotboxArgs(plots=['torque_x', 'torque_y', 'torque_z'],
                                   labels={'left': 'tau(Nm)', 'bottom': 'Time (s)'},
                                   time_window=time_window_length)
        second_row = [vx_plots, vy_plots, vz_plots, torque_plot]

        # define third row
        phi_plots = PlotboxArgs(plots=['phi', 'phi_e', 'phi_c'],
                                labels={'left': 'phi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        theta_plots = PlotboxArgs(plots=['theta', 'theta_e', 'theta_c'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        psi_plots = PlotboxArgs(plots=['psi', 'psi_e', 'psi_c'],
                                labels={'left': 'psi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        gimbal_plot = PlotboxArgs(plots=['azimuth', 'elevation'],
                                   labels={'left': 'gimbal(deg)', 'bottom': 'Time (s)'},
                                   time_window=time_window_length)
        third_row = [phi_plots, theta_plots, psi_plots, gimbal_plot]

        # define fourth row
        omega_x_plots = PlotboxArgs(plots=['omega_x', 'omega_x_e'],
                              labels={'left': 'omega_x(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        omega_y_plots = PlotboxArgs(plots=['omega_y', 'omega_y_e'],
                              labels={'left': 'omega_y(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        omega_z_plots = PlotboxArgs(plots=['omega_z', 'omega_z_e'],
                              labels={'left': 'omega_z(deg)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        gimbal_input_plot = PlotboxArgs(plots=['azimuth_command', 'elevation_command'],
                                   labels={'left': 'gimbal_command', 'bottom': 'Time (s)'},
                                   time_window=time_window_length)
        fourth_row = [omega_x_plots, omega_y_plots, omega_z_plots, gimbal_input_plot]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row,
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state', ['pn', 'pe', 'h',
                                                        'velx', 'vely', 'velz',
                                                        'phi', 'theta', 'psi',
                                                        'omega_x', 'omega_y', 'omega_z',
                                                        'azimuth', 'elevation'])
        self.plotter.define_input_vector('estimated_state', ['pn_e', 'pe_e', 'h_e',
                                                             'velx_e', 'vely_e', 'velz_e',
                                                             'phi_e', 'theta_e', 'psi_e',
                                                             'omega_x_e', 'omega_y_e', 'omega_z_e'])
        self.plotter.define_input_vector('commanded_state', ['pn_c', 'pe_c', 'h_c',
                                                             'velx_c', 'vely_c', 'velz_c',
                                                             'phi_c', 'theta_c', 'psi_c'])
        self.plotter.define_input_vector('delta', ['force', 'torque_x', 'torque_y', 'torque_z', 'azumith_c', 'elevation_c'])
        # plot timer
        self.time = 0.

    def update(self, true_state, estimated_state, commanded_state, delta, ts):
        ## Add the state data in vectors
        # the order has to match the order in lines 72-76
        phi, theta, psi = rotation_to_euler(true_state.rot)
        true_state_list = [true_state.pos.item(0), true_state.pos.item(1), -true_state.pos.item(2),
                           true_state.vel.item(0), true_state.vel.item(1), true_state.vel.item(2),
                           phi, theta, psi,
                           true_state.omega.item(0), true_state.omega.item(1), true_state.omega.item(2),
                           true_state.gimbal.item(0), true_state.gimbal.item(1)]
        phi, theta, psi = rotation_to_euler(estimated_state.rot)
        estimated_state_list = [estimated_state.pos.item(0), estimated_state.pos.item(1), -estimated_state.pos.item(2),
                           estimated_state.vel.item(0), estimated_state.vel.item(1), estimated_state.vel.item(2),
                           phi, theta, psi,
                           estimated_state.omega.item(0), estimated_state.omega.item(1), estimated_state.omega.item(2)]
        phi, theta, psi = rotation_to_euler(commanded_state.rot)
        commanded_state_list = [commanded_state.pos.item(0), commanded_state.pos.item(1), -commanded_state.pos.item(2),
                           commanded_state.vel.item(0), commanded_state.vel.item(1), commanded_state.vel.item(2),
                           phi, theta, psi]
        delta_list = [delta.force.item(0), delta.torque.item(0), delta.torque.item(1), delta.torque.item(2), delta.gimbal_input.item(0), delta.gimbal_input.item(1)]
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)
        self.plotter.add_vector_measurement('estimated_state', estimated_state_list, self.time)
        self.plotter.add_vector_measurement('commanded_state', commanded_state_list, self.time)
        self.plotter.add_vector_measurement('delta', delta_list, self.time)

        # Update and display the plot
        self.plotter.update_plots()

        # increment time
        self.time += ts




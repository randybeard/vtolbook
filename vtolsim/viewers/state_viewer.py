"""
data_viewer

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
"""
from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

import sys
sys.path.append('..')
from tools.rotations import Quaternion2Euler


class stateViewer:
    def __init__(self):
        time_window_length=200
        self.plotter = Plotter(plotting_frequency=100, # refresh plot every 100 time steps
                               time_window=time_window_length, # plot last time_window seconds of data
                               window_title = "States") # name the window
        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn', 'pn_e', 'pn_c'],
                               labels={'left': 'pn(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        pe_plots = PlotboxArgs(plots=['pe', 'pe_e', 'pe_c'],
                               labels={'left': 'pe(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        h_plots = PlotboxArgs(plots=['pz', 'pz_e', 'pz_c'],
                              labels={'left': 'h(m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        first_row = [pn_plots, pe_plots, h_plots]

        # define second row
        u_plots = PlotboxArgs(plots=['u', 'u_e', 'u_c'],
                               labels={'left': 'u(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        v_plots = PlotboxArgs(plots=['v', 'v_e', 'v_c'],
                               labels={'left': 'v(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        w_plots = PlotboxArgs(plots=['w', 'w_e', 'w_c'],
                               labels={'left': 'u(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)

        second_row = [u_plots, v_plots, w_plots]

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
        third_row = [phi_plots, theta_plots, psi_plots]

        # define fourth row
        p_plots = PlotboxArgs(plots=['p', 'p_e', 'p_c'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        q_plots = PlotboxArgs(plots=['q', 'q_e', 'q_c'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        r_plots = PlotboxArgs(plots=['r', 'r_e', 'r_c'],
                              labels={'left': 'r(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        fourth_row = [p_plots, q_plots, r_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state',      ['pn', 'pe', 'pz', 'u', 'v', 'w', 'phi', 'theta', 'psi', 'p', 'q', 'r'])
        self.plotter.define_input_vector('estimated_state', ['pn_e', 'pe_e', 'pz_e', 'u_e', 'v_e', 'w_e', 'phi_e', 'theta_e', 'psi_e', 'p_e', 'q_e', 'r_e'])
        self.plotter.define_input_vector('desired_state',   ['pn_c', 'pe_c', 'pz_c', 'u_c', 'v_c', 'w_c', 'phi_c', 'theta_c', 'psi_c'])
        self.plotter.define_input_vector('commanded_rates', ['p_c', 'q_c', 'r_c'])
        # plot timer
        self.time = 0.

    def update(self, true_state, estimated_state, commanded_state, commanded_rates, ts):

        #convert quaternions to euler angles

        phi, theta, psi = Quaternion2Euler(true_state[6:10])
        true_st_euler = np.concatenate((true_state[0:6], np.array([[phi, theta, psi]]).T, true_state[10:13]))
        true_st_euler = np.squeeze(true_st_euler)

        phi, theta, psi = Quaternion2Euler(estimated_state[6:10])
        estimated_st_euler = np.concatenate((estimated_state[0:6], np.array([[phi, theta, psi]]).T, estimated_state[10:13]))
        estimated_st_euler = np.squeeze(estimated_st_euler)

        phi, theta, psi = Quaternion2Euler(commanded_state[6:10])
        commanded_st_euler = np.concatenate((commanded_state[0:6], np.array([[phi, theta, psi]]).T))
        commanded_st_euler = np.squeeze(commanded_st_euler)

        self.plotter.add_vector_measurement('true_state', true_st_euler, self.time)
        self.plotter.add_vector_measurement('estimated_state', estimated_st_euler, self.time)
        self.plotter.add_vector_measurement('desired_state', commanded_st_euler, self.time)
        # self.plotter.add_vector_measurement('commanded_rates', commanded_rates, self.time)

        self.tick()

        # increment time
        self.time += ts

    def tick(self):
        # Update and display the plot
        self.plotter.update_plots()
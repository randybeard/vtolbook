"""
thrust_torque_viewer
plot the desired and actual thrust and torque
"""
from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

class ThrustTorqueViewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=100, # refresh plot every 100 time steps
                               time_window=time_window_length, # plot last time_window seconds of data
                               window_title = "Thrust and Torques") # name the window
        # set up the plot window
        # define first row
        thrust_x_plots = PlotboxArgs(plots=['thrust_x', 'thrust_x_d'],
                               labels={'left': 'Force X', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        thrust_z_plots = PlotboxArgs(plots=['thrust_z', 'thrust_z_d'],
                               labels={'left': 'Force Z', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        thrust_angle_plots = PlotboxArgs(plots=['thrust_ang', 'thrust_ang_d'],
                               rad2deg=True,
                               labels={'left': 'Force angle (deg)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        first_row = [thrust_x_plots, thrust_z_plots, thrust_angle_plots]

        # define second row
        l_plots = PlotboxArgs(plots=['l', 'l_d'],
                              labels={'left': 'l (N-m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        m_plots = PlotboxArgs(plots=['m', 'm_d'],
                              labels={'left': 'm (N-m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        n_plots = PlotboxArgs(plots=['n', 'n_d'],
                              labels={'left': 'n (N-m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        second_row = [l_plots, m_plots, n_plots]

        plots = [first_row,
                 second_row ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('thrust_torque',
                ['thrust_x', 'thrust_z', 'thrust_ang',
                    'l', 'm', 'n'])

        self.plotter.define_input_vector('thrust_torque_des',
                ['thrust_x_d', 'thrust_z_d', 'thrust_ang_d',
                    'l_d', 'm_d', 'n_d'])

        # plot timer
        self.time = 0.

    def update(self, thrust_torque, thrust_torque_des, ts):

        force_ang = np.arctan2(thrust_torque.item(1),thrust_torque.item(0))
        thrust_torque = np.squeeze(thrust_torque)
        thrust_torque = np.insert(thrust_torque, 2, force_ang)
        self.plotter.add_vector_measurement('thrust_torque', thrust_torque, self.time)

        force_ang_d = np.arctan2(thrust_torque_des.item(1),thrust_torque_des.item(0))
        thrust_torque_des = np.squeeze(thrust_torque_des)
        thrust_torque_des = np.insert(thrust_torque_des, 2, force_ang)
        self.plotter.add_vector_measurement('thrust_torque_des', thrust_torque_des, self.time)

        self.tick()

        # increment time
        self.time += ts

    def tick(self):
        # Update and display the plot
        self.plotter.update_plots()



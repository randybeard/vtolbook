"""
controls_viewer
plot the control inputs

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
"""
from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

class controlsViewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=100, # refresh plot every 100 time steps
                               time_window=time_window_length, # plot last time_window seconds of data
                               window_title = "Controls") # name the window
        # set up the plot window
        # define first row
        motor_rear_plots = PlotboxArgs(plots=['throttle_rear'],
                               labels={'left': 'delta_t', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        motor_right_plots = PlotboxArgs(plots=['throttle_right'],
                               labels={'left': 'delta_t', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        motor_left_plots = PlotboxArgs(plots=['throttle_left'],
                               labels={'left': 'delta_t', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        first_row = [motor_right_plots, motor_left_plots, motor_rear_plots]

        # define second row
        # motor_rear_angle = PlotboxArgs(plots=['servo_rear'],
                               # rad2deg=True,
                               # labels={'left': 'angle (deg)', 'bottom': 'Time (s)'},
                               # time_window=time_window_length)
        motor_right_angle = PlotboxArgs(plots=['servo_right'],
                               rad2deg=True,
                               labels={'left': 'angle (deg)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        motor_left_angle = PlotboxArgs(plots=['servo_left'],
                               rad2deg=True,
                               labels={'left': 'angle (deg)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        second_row = [motor_right_angle, motor_left_angle]

        #define third row
        elevon_right_plot = PlotboxArgs(plots=['elevon_right'],
                               rad2deg=True,
                               labels={'left': 'angle (deg)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        elevon_left_plot = PlotboxArgs(plots=['elevon_left'],
                               rad2deg=True,
                               labels={'left': 'angle (deg)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        third_row = [elevon_right_plot, elevon_left_plot]

        plots = [first_row,
                 second_row,
                 third_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('controls', 
                ['throttle_rear', 'throttle_right', 'throttle_left', 
                    'servo_right', 'servo_left',
                    'elevon_right', 'elevon_left'])

        # plot timer
        self.time = 0.

    def update(self, controls, ts):
        ## Add the state data in vectors
        # the order has to match the order in lines 72-76
        controls_list = [controls.throttle_rear, 
                controls.throttle_right, 
                controls.throttle_left,
                controls.servo_right,
                controls.servo_left,
                controls.elevon_right,
                controls.elevon_left]

        self.plotter.add_vector_measurement('controls', controls_list, self.time)

        self.tick()

        # increment time
        self.time += ts

    def tick(self):
        # Update and display the plot
        self.plotter.update_plots()

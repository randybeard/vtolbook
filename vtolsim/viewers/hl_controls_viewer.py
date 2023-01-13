"""
hl_controls_viewer
plot the high level controls


"""
from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

class HLControlsViewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=100, # refresh plot every 100 time steps
                               time_window=time_window_length, # plot last time_window seconds of data
                               window_title = "High Level Controls") # name the window
        # set up the plot window
        # define first row
        force_x_plots = PlotboxArgs(plots=['force_x', 'force_x_d'],
                               labels={'left': 'Force X', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        force_z_plots = PlotboxArgs(plots=['force_z', 'force_z_d'],
                               labels={'left': 'Force Z', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        force_angle_plots = PlotboxArgs(plots=['force_ang', 'force_ang_d'],
                               rad2deg=True,
                               labels={'left': 'Force angle (deg)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        first_row = [force_x_plots, force_z_plots, force_angle_plots]

        # define second row
        p_plots = PlotboxArgs(plots=['p', 'p_d'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        q_plots = PlotboxArgs(plots=['q', 'q_d'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        r_plots = PlotboxArgs(plots=['r', 'r_d'],
                              labels={'left': 'r(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        second_row = [p_plots, q_plots, r_plots]

        plots = [first_row,
                 second_row ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('hl_controls', 
                ['force_x', 'force_z', 'force_ang', 
                    'p', 'q', 'r'])

        self.plotter.define_input_vector('hl_controls_des', 
                ['force_x_d', 'force_z_d', 'force_ang_d', 
                    'p_d', 'q_d', 'r_d'])

        # plot timer
        self.time = 0.

    def update(self, hl_controls, hl_controls_des, ts):
        
        force_ang = np.arctan2(hl_controls.item(1),hl_controls.item(0))
        hl_controls = np.squeeze(hl_controls)
        hl_controls = np.insert(hl_controls, 2, force_ang)
        self.plotter.add_vector_measurement('hl_controls', hl_controls, self.time)

        force_ang_d = np.arctan2(hl_controls_des.item(1),hl_controls_des.item(0))
        hl_controls_des = np.squeeze(hl_controls_des)
        hl_controls_des = np.insert(hl_controls_des, 2, force_ang)
        self.plotter.add_vector_measurement('hl_controls_des', hl_controls_des, self.time)

        self.tick()

        # increment time
        self.time += ts

    def tick(self):
        # Update and display the plot
        self.plotter.update_plots()



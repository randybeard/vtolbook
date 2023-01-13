import sys
sys.path.append('..')
import numpy as np
import parameters.landing_parameters_precomputed as landing

######################################################################################
                #   sample times, etc
######################################################################################
ts_simulation = 0.01  # smallest time step for simulation
start_time = 0.0  # start time for simulation
end_time = landing.sim_time

ts_plotting = 1.0  # refresh rate for plots

ts_video = 1.0#0.1  # write rate for video

ts_control = ts_simulation  # sample rate for the controller

show_airsim = False

import sys
sys.path.append('..')
import numpy as np

######################################################################################
                #   sample times, etc
######################################################################################
ts_simulation = 0.003  # smallest time step for simulation -- AirSim is hard-coded with 3 ms timestep
start_time = 0.0  # start time for simulation
end_time = 70.
loop_rate = 100.0  # Main loop execution rate in Hz
dt_loop_target = 1/loop_rate * 0.992

# False will not make any calls to airsim, for debugging
use_airsim = True

# Using simContinueForTime in loop will start a pause-play cycle for airsim
# must call client.simPause(False) for airsim to play normally again
use_simContinueForTime = False

ts_plotting = 1.0  # refresh rate for plots
ts_control = ts_simulation  # sample rate for the controller

# False turns off the plots/viewers
plots = False
viewer = False
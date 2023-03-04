# Simulation Parameter File
import numpy as np

# environment
size_N = 200  # meters
size_E = 200  # meters

# intruder initial conditions
height_0 = 100 # m
v_0 = 5  # m/s
intruder_p0 = np.array([[size_N/4, -size_E/2 + size_E/20, -height_0]]).T
intruder_v0 = np.array([[0, v_0, 0]]).T
intruder_covar0 = 30 * np.array([[1, 0], [0, 1]])

# ownship initial conditions
ownship_p0 = np.array([[-size_N/2, 0, -height_0]]).T
ownship_v0 = np.array([[1.5*v_0, 0, 0]]).T

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 50.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate

# drawing parameters
uav_size = 1



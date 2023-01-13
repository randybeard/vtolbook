"""
differential_flatness_parameters
    parameters for controlling differential flatness behavior
"""

import sys
sys.path.append('..')
import numpy as np

alpha_max = np.radians(10.) #radians

k_w = 1. + .2 # must be > 1
u_max_alpha = 20. # m/s, the point where alpha_max is hit (if w = 0)

k_u = np.exp(1./(2.*u_max_alpha**2)) 
lnk = np.log(k_u)
mix_u_max = np.sqrt(1./(2.*lnk))*((k_u)**(-1./(2.*lnk)))


"""
Parameters for pitch/thrust optimization
"""

import sys
sys.path.append('..')
import numpy as np

theta_min = np.deg2rad(-15)
theta_max = np.deg2rad(15)

alpha_aero_min = np.deg2rad(-15)
alpha_aero_max = np.deg2rad(15)

xi_T_max = np.pi/2
xi_T_min = np.deg2rad(-15)

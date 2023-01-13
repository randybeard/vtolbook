"""
Parameters for geometric controller
"""

import sys
sys.path.append('..')
import numpy as np
from geometric_control.optimal_pitch import AERO_TYPE, OPTIMAL_PITCH_METHOD

# decent
# Kp = np.diag([4.0, 4.0, 6.])
# Kd = np.diag([3.0, 3.0, 3.0])
# Ki = 0.*np.diag([.1, .1, .1])

Kp = np.diag([3.0, 3.0, 5.0])
Kd = np.diag([2.5, 2.5, 2.5])
Ki = 0.*np.diag([.1, .1, .1])
omega_Kp = 5.*np.diag([1., 1., 1.])

perr_d0_sat = 5.

delta_theta_max = .0005 # rad/step

aero_type = AERO_TYPE.BLENDED_2

optimal_pitch_method = OPTIMAL_PITCH_METHOD.Sampled

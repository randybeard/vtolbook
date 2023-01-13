"""
lqr_parameters
    Parameters for the full state lqr controller
"""

import sys
sys.path.append('..')
import numpy as np


# step used for numeric integration/differentiation
epsilon = .001

# Q and R cost matrices

Q = \
np.array([[1/10., 0., 0., 0., 0., 0., 0., 0., 0.],  # [0]  north position
          [0., 1/10., 0., 0., 0., 0., 0., 0., 0.],  # [1]  east position
          [0., 0., 1/10., 0., 0., 0., 0., 0., 0.],  # [2]  down position
          [0., 0., 0., 1/10., 0., 0., 0., 0., 0.],  # [3]  velocity along body x-axis
          [0., 0., 0., 0., 1/10., 0., 0., 0., 0.],  # [4]  velocity along body y-axis
          [0., 0., 0., 0., 0., 1/10., 0., 0., 0.],  # [5]  velocity along body z-axis
          [0., 0., 0., 0., 0., 0., 2., 0., 0.],  # [6]  q_tilde0
          [0., 0., 0., 0., 0., 0., 0., 2., 0.],  # [7]  q_tilde1
          [0., 0., 0., 0., 0., 0., 0., 0., 2.]]) # [8]  q_tilde2

R = \
np.array([[1., 0., 0., 0., 0.], # ax
          [0., 1., 0., 0., 0.], # az
          [0., 0., 10., 0., 0.], # p
          [0., 0., 0., 10., 0.], # q
          [0., 0., 0., 0., 150.]])# r

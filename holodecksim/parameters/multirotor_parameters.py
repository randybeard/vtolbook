import sys
sys.path.append('..')
import numpy as np
from tools.rotations import euler_to_rotation

######################################################################################
                #   Initial Conditions
######################################################################################
#   Initial conditions for multirotor
pos0 = np.array([[0.], [0.], [0.]])  # initial position in inertial frame
vel0 = np.array([[0.], [0.], [0.]])  # initial velocity in inertial frame
rot0 = euler_to_rotation(phi=0., theta=0., psi=0.)  # initial rotation from body to inertial
omega0 = np.array([[0.], [0.], [0.]])  # initial angular velocity
gimbal0 = np.array([[0.], [0.], [0.]])  # gimbal azimuth, elevation, roll angle

######################################################################################
                #   Physical Parameters
######################################################################################
gravity = 9.8  # m/s^2
mass = 3.  # kg
J = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 1.759]])  # Nm
Jinv = np.linalg.inv(J)

Cd = 0.3  # induced drag coefficient

# The gimbal dynamics are assumed to be gimbal_dot = k_gimbal * (gimbal_input - gimbal)
k_gimbal = 10  # gimbal gain.


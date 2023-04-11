import numpy as np
from tools.rotations import euler_to_rotation


######################################################################################
                #   Initial Conditions
######################################################################################
#   Initial conditions for MAV
pos0 = np.array([[0.], [0.], [0.]])  # initial position in inertial frame
vel0 = np.array([[0.], [0.], [0.]])  # initial velocity in inertial frame
rot0 = euler_to_rotation(phi=0., theta=0., psi=0.)  # initial rotation from body to inertial
omega0 = np.array([[0.], [0.], [0.]])  # initial angular velocity

######################################################################################
                #   Physical Parameters
######################################################################################
gravity = 9.8  # m/s^2
mass = 3.  # kg
Tmax = 40.
J = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 1.759]])  # Nm
Jinv = np.linalg.inv(J)

rho = 1.2682
e = 0.9

######################################################################################
                #   Propeller thrust / torque parameters (see addendum by McLain)
######################################################################################
# Prop parameters
D_prop = 20*(0.0254)     # prop diameter in m

# Motor parameters
KV = 145.                   # from datasheet RPM/V
KQ = (1. / KV) * 60. / (2. * np.pi)  # KQ in N-m/A, V-s/rad
R_motor = 0.042              # ohms
i0 = 1.5                     # no-load (zero-torque) current (A)


# Inputs
ncells = 12.
V_max = 3.7 * ncells  # max voltage for specified number of battery cells

# Coeffiecients from prop_data fit
C_Q2 = -0.01664
C_Q1 = 0.004970
C_Q0 = 0.005230
C_T2 = -0.1079
C_T1 = -0.06044
C_T0 = 0.09357

"""
convergence_parameters
    - This file contains the aerodynamic parameters for the convergence aircraft.
    (https://www.horizonhobby.com/product/airplanes/airplanes-14501--1/bind-n-fly/convergence-vtol-bnf-basic-efl11050)
    - The current numbers are just the aerosonde parameters, with a few modifications
    - Need to do some sysid to correctly identify the convergence parameters.

    - Update history:
        5/8/2019 - R.W. Beard
"""

import sys
sys.path.append('..')
import numpy as np
from tools.rotations import Euler2Quaternion, Quaternion2Rotation
import parameters.landing_parameters_precomputed as landing

######################################################################################
                #   Initial Conditions
######################################################################################
#   Initial conditions for MAV
pn0 = 0.  # initial north position
pe0 = 0. # initial east position
pd0 = 0.  # initial down position
u0 = landing.u0 # initial velocity along body x-axis
v0 = 0.  # initial velocity along body y-axis
w0 = landing.w0 # initial velocity along body z-axis
phi0 = 0. # initial roll angle
theta0 =  landing.theta  # initial pitch angle
psi0 = 0. # initial yaw angle
p0 = 0  # initial roll rate
q0 = 0  # initial pitch rate
r0 = 0  # initial yaw rate
Va0 = np.sqrt(u0**2+v0**2+w0**2)
#   Quaternion State
e = Euler2Quaternion(phi0, theta0, psi0)
e0 = e.item(0)
e1 = e.item(1)
e2 = e.item(2)
e3 = e.item(3)
right_rotor = 1.3837 #np.radians(90)
left_rotor = 1.4544 #np.radians(90)
Tmax = 22.6

######################################################################################
                #   Physical Parameters
######################################################################################
######Aeorsonde#######
# mass = 11 #kg
# Jx = 0.8244 #kg m^2
# Jy = 1.135
# Jz = 1.759
# Jxz = 0.1204
# S_wing = 0.55
# b = 2.8956
# c = 0.18994

######Convergence#######
mass = 1.0 #kg
Jx = 0.0165#0.008383 #kg m^2
Jy = 0.025#0.006114
Jz = 0.0282#0.01435
Jxz = 0.000048#0.000024
J = np.array([[Jx, 0, -Jxz], [0, Jy, 0], [-Jxz, 0, Jz]])
S_wing = 0.2589#0.19#0.55
b = 1.4224#0.65#2.8956
c = 0.3305#0.25#0.18994

########Zagi########
# mass = 1.53 #kg
# Jx = 0.1147 #kg m^2
# Jy = 0.0576
# Jz = 0.1712
# Jxz = 0.0015
# S_wing = 0.2589
# b = 1.4224
# c = 0.3302

S_prop = 0.2027
rho = 1.2682
e = 0.9
e_oswald = e
AR = (b**2) / S_wing
AR_wing = AR
gravity = 9.81

# physical position of rotors in units of meters
rear_rotor_pos = np.array([[-0.24], [0.0], [0.0]])
right_rotor_pos = np.array([[0.12], [0.2], [0.0]])
left_rotor_pos = np.array([[0.12], [-0.2], [0.0]])

######################################################################################
                #   Longitudinal Coefficients
######################################################################################
######Aeorsonde#######
# C_L_0 = 0.23
# C_D_0 = 0.043
# C_m_0 = 0.0135
# C_L_alpha = 5.61
# C_D_alpha = 0.03
# C_m_alpha = -2.74
# C_L_q = 7.95
# C_D_q = 0.0
# C_m_q = -38.21
# C_L_delta_e = 0.13
# C_D_delta_e = 0.0135
# C_m_delta_e = -0.99

######Convergence#######
C_L_0 = 0.005
C_D_0 = 0.0022
C_m_0 = 0.0
C_L_alpha = 2.819
C_D_alpha = 0.03
C_m_alpha = -0.185
C_L_q = 3.242
C_D_q = 0.0
C_m_q = -1.093
C_L_delta_e = 0.2
C_D_delta_e = 0.005
C_m_delta_e = -0.05

######Zagi#########
# C_L_0 = 0.09167
# C_D_0 = 0.01631
# C_m_0 = -0.02338
# C_L_alpha = 3.5016
# C_D_alpha = 0.2108
# C_m_alpha = -0.5675
# C_L_q = 2.8932
# C_D_q = 0.0
# C_m_q = -1.3990
# C_L_delta_e = 0.2724
# C_D_delta_e = 0.3045
# C_m_delta_e = -0.3254

M = 50.0
alpha0 = np.deg2rad(15) #0.47
epsilon = 0.16
C_D_p = 0.015# Parasitic Drag


######################################################################################
                #   Lateral Coefficients
######################################################################################
C_Y_0 = 0.0
C_ell_0 = 0.0
C_n_0 = 0.0

######Aeorsonde#######
# C_Y_beta = -0.98
# C_ell_beta = -0.13
# C_n_beta = 0.073
# C_Y_p = 0.0
# C_ell_p = -0.51
# C_n_p = 0.069
# C_Y_r = 0.0
# C_ell_r = 0.25
# C_n_r = -0.095
# C_Y_delta_a = 0.075
# C_ell_delta_a = 0.17
# C_n_delta_a = -0.011
# C_Y_delta_r = 0.19
# C_ell_delta_r = 0.0024
# C_n_delta_r = -0.069

######Convergence#######
C_Y_beta = -0.318
C_ell_beta = -0.032
C_n_beta = 0.112
C_Y_p = 0.078
C_ell_p = -0.207
C_n_p = -0.053
C_Y_r = 0.288
C_ell_r = 0.036
C_n_r = -0.104
C_Y_delta_a = 0.000536
C_ell_delta_a = 0.018
C_n_delta_a = -0.00328#-0.011
C_Y_delta_r = 0.0
C_ell_delta_r = 0.0
C_n_delta_r = 0.0

########Zagi#########
# C_Y_beta = -0.07359
# C_ell_beta = -0.02854
# C_n_beta = -0.00040
# C_Y_p = 0.0
# C_ell_p = -0.3209
# C_n_p = -0.01297
# C_Y_r = 0.0
# C_ell_r = 0.03066
# C_n_r = -0.00434
# C_Y_delta_a = 0.0
# C_ell_delta_a = 0.1682
# C_n_delta_a = -0.00328
# C_Y_delta_r = 0.0
# C_ell_delta_r = 0.0
# C_n_delta_r = 0.0

######################################################################################
                #   Propeller thrust / torque parameters (see addendum by McLain)
######################################################################################
######Aeorsonde#######
# # Prop parameters
D_prop = 20*(0.0254)     # prop diameter in m

# Motor parameters
K_V = 145.                   # from datasheet RPM/V
KQ = (1. / K_V) * 60. / (2. * np.pi)  # KQ in N-m/A, V-s/rad
R_motor = 0.042              # ohms
i0 = 1.5                     # no-load (zero-torque) current (A)

k_force = 65.0
k_moment = 5.0

# Inputs
# ncells = 12.
# V_max = 3.7 * ncells  # max voltage for specified number of battery cells

# Coeffiecients from prop_data fit
C_Q2 = -0.01664
C_Q1 = 0.004970
C_Q0 = 0.005230
C_T2 = -0.1079
C_T1 = -0.06044
C_T0 = 0.09357

############################# Rear ##################################
######Convergence#######
# Prop parameters
D_prop_rear = 5.5*(0.0254)    # prop diameter in m

# Motor parameters
K_V_rear = 1550.                   # from datasheet RPM/V
KQ_rear = (1. / K_V_rear) * 60. / (2. * np.pi)  # KQ in N-m/A, V-s/rad
R_motor_rear = 0.4              # ohms
i0_rear = 0.6                     # no-load (zero-torque) current (A)

# Inputs
ncells = 6.
V_max = 3.7 * ncells  # max voltage for specified number of battery cells

# Coeffiecients from prop_data fit
C_Q2_rear = -0.0368
C_Q1_rear = 0.0292
C_Q0_rear = 0.0216
C_T2_rear = -0.1921
C_T1_rear = 0.0505
C_T0_rear = 0.2097

############################# Front Left/Right ##################################
######Convergence#######
# Prop parameters
D_prop_front = 7.0*(0.0254)#20*(0.0254)     # prop diameter in m

# Motor parameters
K_V_front = 1450.                   # from datasheet RPM/V
KQ_front = (1. / K_V_front) * 60. / (2. * np.pi)  # KQ in N-m/A, V-s/rad
R_motor_front = 0.3              # ohms
i0_front = 0.83                     # no-load (zero-torque) current (A)

# Coeffiecients from prop_data fit
C_Q2_front = -0.0216
C_Q1_front = 0.0129
C_Q0_front = 0.0088
C_T2_front = -0.1480
C_T1_front = 0.0144
C_T0_front = 0.1167

######################################################################################
                #   Front motor servo parameters
######################################################################################

#First order servo model alpha_dot = k*(delta-alpha)
k_servo = 10.0
servo_min = np.radians(0.0)
servo_max = np.radians(115.0)

######################################################################################
                #   Elevon parameters
######################################################################################

elevon_min = np.radians(-45.0)
elevon_max = np.radians(45.0)

######################################################################################
                #   Calculation Variables
######################################################################################
#   gamma parameters pulled from page 36 (dynamics)
gamma = Jx * Jz - (Jxz**2)
gamma1 = (Jxz * (Jx - Jy + Jz)) / gamma
gamma2 = (Jz * (Jz - Jy) + (Jxz**2)) / gamma
gamma3 = Jz / gamma
gamma4 = Jxz / gamma
gamma5 = (Jz - Jx) / Jy
gamma6 = Jxz / Jy
gamma7 = ((Jx - Jy) * Jx + (Jxz**2)) / gamma
gamma8 = Jx / gamma

#   C values defines on pag 62
C_p_0         = gamma3 * C_ell_0      + gamma4 * C_n_0
C_p_beta      = gamma3 * C_ell_beta   + gamma4 * C_n_beta
C_p_p         = gamma3 * C_ell_p      + gamma4 * C_n_p
C_p_r         = gamma3 * C_ell_r      + gamma4 * C_n_r
C_p_delta_a    = gamma3 * C_ell_delta_a + gamma4 * C_n_delta_a
C_p_delta_r    = gamma3 * C_ell_delta_r + gamma4 * C_n_delta_r
C_r_0         = gamma4 * C_ell_0      + gamma8 * C_n_0
C_r_beta      = gamma4 * C_ell_beta   + gamma8 * C_n_beta
C_r_p         = gamma4 * C_ell_p      + gamma8 * C_n_p
C_r_r         = gamma4 * C_ell_r      + gamma8 * C_n_r
C_r_delta_a    = gamma4 * C_ell_delta_a + gamma8 * C_n_delta_a
C_r_delta_r    = gamma4 * C_ell_delta_r + gamma8 * C_n_delta_r

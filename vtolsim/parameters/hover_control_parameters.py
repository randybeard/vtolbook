import sys
sys.path.append('..')
import numpy as np
import dynamics.transfer_function_coef as TF
import parameters.convergence_parameters as VTOL

gravity = VTOL.gravity  # gravity constant
rho = VTOL.rho  # density of air
sigma = 0.05  # low pass filter gain for derivative
Va0 = TF.Va_trim

#----------roll loop-------------
# get transfer function data for delta_a to phi
# tr_roll = 0.3
# wn_roll = 2.2/tr_roll
# zeta_roll = 0.707
# roll_kp = VTOL.Jx*wn_roll**2
# roll_ki = 0.0
# roll_kd = VTOL.Jx*2.0*zeta_roll*wn_roll

roll_kp = 0.18
roll_ki = 0.0
roll_kd = 0.07

#----------pitch loop-------------
# tr_pitch = 0.5
# wn_pitch = 2.2/tr_pitch
# zeta_pitch = 0.7
# pitch_kp = VTOL.Jy*wn_pitch**2
# pitch_ki = 0.0
# pitch_kd = VTOL.Jy*2*zeta_pitch*wn_pitch

pitch_kp = 0.15
pitch_ki = 0.0
pitch_kd = 0.05

#----------altitude loop-------------
# tr_altitude = 3.0
# wn_altitude = 2.2/tr_altitude
# zeta_altitude = 1.3
# altitude_kp = -wn_altitude**2
# altitude_ki = -0.1
# altitude_kd = -2*zeta_altitude*wn_altitude
# altitude_zone = 4.0  # moving saturation limit around current altitude

altitude_kp = -2.5
altitude_ki = -0.3
altitude_kd = -1.2
altitude_zone = 1.0  # moving saturation limit around current altitude

#-----------pn loop---------------
# tr_pn = 2.0
# wn_pn = 2.2/tr_pn
# zeta_pn = 1.3
# pn_kp = wn_pn**2
# pn_ki = 0.4
# pn_kd = 2*zeta_pn*wn_pn

pn_kp = 1.5
pn_ki = 0.2
pn_kd = 2.5

#-----------pe loop---------------
# tr_pe = 3.0
# wn_pe = 2.2/tr_pe
# zeta_pe = 0.9
# pe_kp = wn_pe**2
# pe_ki = 0.01
# pe_kd = 2*zeta_pe*wn_pe

pe_kp = 0.55
pe_ki = 0.01
pe_kd = 1.3

#---------yaw loop---------------
# tr_yaw = 0.5
# wn_yaw = 2.2/tr_yaw
# zeta_yaw = 0.7
# yaw_kp = VTOL.Jz*wn_yaw**2
# yaw_ki = 0.01
# yaw_kd = VTOL.Jz*2*zeta_yaw*wn_yaw

yaw_kp = -0.5
yaw_ki = -0.05
yaw_kd = -0.5

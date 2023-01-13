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
wn_roll = 4
zeta_roll = 2.0
roll_kp = wn_roll**2/TF.a_phi2
roll_kd = (2.0 * zeta_roll * wn_roll - TF.a_phi1) / TF.a_phi2

#----------course loop-------------
wn_course = wn_roll / 20.0
zeta_course = 1.0
course_kp = 2 * zeta_course * wn_course * Va0 / gravity
course_ki = wn_course**2 * Va0 / gravity

#----------sideslip loop-------------
# wn_sideslip = 0.5
# zeta_sideslip = 5.0
# sideslip_ki = wn_sideslip**2 / TF.a_beta2
# sideslip_kp = (2 * zeta_sideslip * wn_sideslip - TF.a_beta1) / TF.a_beta2

#----------yaw damper-------------
# yaw_damper_tau_r = 0.5
# yaw_damper_kp = 0.5

#----------pitch loop-------------
wn_pitch = 50.0
zeta_pitch = 0.707
pitch_kp = (wn_pitch**2 - TF.a_theta2) / TF.a_theta3
pitch_kd = (2.0 * zeta_pitch * wn_pitch - TF.a_theta1) / TF.a_theta3
K_theta_DC = pitch_kp * TF.a_theta3 / (TF.a_theta2 + pitch_kp * TF.a_theta3)

#----------altitude loop-------------
wn_altitude = wn_pitch / 50.0
zeta_altitude = 1.0
altitude_kp = 2.0 * zeta_altitude * wn_altitude / K_theta_DC / Va0
altitude_ki = wn_altitude**2 / K_theta_DC / Va0
altitude_zone = 10.0  # moving saturation limit around current altitude

#---------airspeed hold using throttle---------------
wn_airspeed_throttle = 3.0
zeta_airspeed_throttle = 2  # 0.707
airspeed_throttle_kp = (2.0 * zeta_airspeed_throttle * wn_airspeed_throttle - TF.a_V1) / TF.a_V2
airspeed_throttle_ki = wn_airspeed_throttle**2 / TF.a_V2

import numpy as np

# Low Level Control Rate parameters
ll_p_kp = 0.15
ll_p_ki = 0.03
ll_p_kd = 0.00

ll_q_kp = 0.08
ll_q_ki = 0.03
ll_q_kd = 0.0

ll_r_kp = 0.1
ll_r_ki = 0.03
ll_r_kd = 0.01

# Rate Controller Rate Parameters
p_kp = 0.50
p_ki = 1.00
p_kd = 0.001

q_kp = 0.70
q_ki = 1.00
q_kd = 0.001

r_kp = 0.50
r_ki = 1.00
r_kd = 0.001

#will later be scaled by sig(Va)
#              t_re, t_ri*c(dr), t_ri*s(dr), t_l*c(dl), t_l*s(dl), er, el
mixer = np.array([[   0.0,   1.0,   0.0,   1.0,   0.0,   0.0,   0.0],  #Fx
                  [   1.1,   0.0,   0.9,   0.0,   0.9,   0.0,   0.0],  #Fz
                  [   0.0,   0.0,  -1.0,   0.0,   1.0,  -1.0,   1.0],  #Tx
                  [  -1.0,   0.0,   1.0,   0.0,   1.0,  -1.0,  -1.0],  #Ty
                  [   0.0,  -1.0,   0.0,   1.0,   0.0,   0.0,   0.0]]) #Tz
                  #convention for elevon flaps is down is positive
#
# mixer = np.array([[   0.0,   1.0,   0.0,   1.0,   0.0,   0.0,   0.0],  #Fx
#                   [   1.1,   0.0,   0.9,   0.0,   0.9,   0.0,   0.0],  #Fz
#                   [   0.0,   0.0,  -1.0,   0.0,   1.0,  -1.0,   1.0],  #Tx
#                   [  -1.0,   0.0,   1.0,   0.0,   1.0,  -3.0,  -3.0],  #Ty
#                   [   0.0,  -1.0,   0.0,   1.0,   0.0,   0.0,   0.0]]) #Tz

limits = np.array([[0.0, 0.0, 0.0,   np.radians(0.0),   np.radians(0.0), np.radians(-45.0), np.radians(-45.0)],  #lower limits
                   [1.0, 1.0, 1.0, np.radians(115.0), np.radians(115.0),  np.radians(45.0),  np.radians(45.0)]]) #upper limits

elevon_k = 5000
alpha = 0.99

phi_kp = 4.0
phi_ki = 0.0
phi_kd = 0.2

theta_kp = 4.0
theta_ki = 0.0
theta_kd = 0.2

psi_kp = 0.5
psi_ki = 0.0
psi_kd = 0.1

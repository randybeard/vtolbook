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

num_props_failed = 2 #can be 1 or 2

######################################################################################
                #   Physical Parameters
######################################################################################
gravity = 9.8  # m/s^2
e3 = np.array([[0.], [0.], [1.]])
mass = 0.5  # kg
Tmax = 3.8
Tmin = 0.2
Ixx_t = 3.2e-3
Izz_t = 5.5e-3
Ixx_p = 0
Izz_p = 1.5e-5
Ixx_b = Ixx_t - 4*Ixx_p
Izz_b = Izz_t - 4*Izz_p
l = 0.17
k_f = 6.41e-6
k_t = 1.69e-2
gamma = 2.75e-3
J = np.array([[Ixx_t, 0, 0], [0, Ixx_t, 0], [0, 0, Izz_t]])  # Nm
Jinv = np.linalg.inv(J)

if num_props_failed == 1:
    f4 = 0
    w4 = 0
    p = 0
    rho_p = 0.5
    f1 = 2.05
    f3 = f1
    f2 = f1*rho_p

    wb = np.array([[0., 5.69, 18.89]]).T
    n = np.array([[0., 0.289, 0.958]]).T
    nx = 0
    p = 0

    B = l/Ixx_b * np.array([[0,1,0,0],
                            [1,0,0,0]]).T

elif num_props_failed == 2:
    f2 = 0
    f4 = 0

    f1 = 1/2*mass*gravity
    f3 = f1

    wb = np.array([[0.],[0.],[k_t*mass*gravity/gamma]])
    n = np.array([[0.],[0.],[1.]])
    Rps = 0

    B = l/Ixx_b * np.array([[0.,1.,0.,0.]]).T

w1 = -np.sqrt(f1/k_f)
w2 = np.sqrt(f2/k_f)
w3 = -np.sqrt(f3/k_f)
w4 = np.sqrt(f4/k_f)
eta = 0.707
wn_heading = 2.2/2.2
# wn_altitude = 2.2/20

nz = n.item(2)

r = k_t*k_f/gamma * (w1**2-w2**2+w3**2-w4**2)
a = (Ixx_t - Izz_t)*r/Ixx_b - Izz_p/Ixx_b*(w1 + w2 + w3 + w4)
A = np.array([[0,a,0,0],
              [-a,0,0,0],
              [0,-nz,0,r],
              [nz,0,-r,0]])

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

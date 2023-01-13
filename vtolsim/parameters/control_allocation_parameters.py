import numpy as np

# weighting matrix for minimization of difference between desired and achieved force/torque
K_Tau = np.eye(5)

# weighting matrix for minimization of difference between optimal and necessary actuator setpoint
def K_delta(airspeed):
    return np.eye(7) * 0#* np.array([[1, 1, 10, 3, 3, 0.0, 0.0]]).T * 1e-6 * airspeed**2

# initial actuator guess
init_actuators = np.array([0.7556, 0.7556, 0.9235, 1.3837, 1.4544, 0.0, 0.0])

# minimum-energy actuator setpoints
actuators_desired = np.zeros(7)

# max iterations for nonlinear solver
max_iter = 15
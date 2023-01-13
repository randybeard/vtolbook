import sys
sys.path.append('..')
import numpy as np
import parameters.low_level_parameters as LP
from low_level_controller.pid_control import pidControl


class RateControl:
    def __init__(self, ts_control):
        self.p_ctrl = pidControl(kp=LP.p_kp, ki=LP.p_ki, kd=LP.p_kd, Ts=ts_control, limit=np.inf)
        self.q_ctrl = pidControl(kp=LP.q_kp, ki=LP.q_ki, kd=LP.q_kd, Ts=ts_control, limit=np.inf)
        self.r_ctrl = pidControl(kp=LP.r_kp, ki=LP.r_ki, kd=LP.r_kd, Ts=ts_control, limit=np.inf)


    def update(self, omega_d, omega, Ts=None):
        tau_x_d = self.p_ctrl.update(omega_d[0], omega[0], Ts)
        tau_y_d = self.q_ctrl.update(omega_d[1], omega[1], Ts)
        tau_z_d = self.r_ctrl.update(omega_d[2], omega[2], Ts)

        return np.array([tau_x_d, tau_y_d, tau_z_d]).reshape(-1)

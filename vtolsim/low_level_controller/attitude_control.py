import sys
sys.path.append('..')
import numpy as np
import parameters.low_level_parameters as LP
from low_level_controller.pid_control import pidControl
from message_types.msg_state import msgState
from message_types.msg_controls import msgControls
import math

class attitudeControl:
    def __init__(self, ts_control):
        self.phi_control = pidControl(kp=LP.phi_kp, ki=LP.phi_ki, kd=LP.phi_kd, Ts=ts_control, limit=np.inf)
        self.theta_control = pidControl(kp=LP.theta_kp, ki=LP.theta_ki, kd=LP.theta_kd, Ts=ts_control, limit=np.inf)
        self.psi_control = pidControl(kp=LP.psi_kp, ki=LP.psi_ki, kd=LP.psi_kd, Ts=ts_control, limit=np.inf)

    def update(self, cmd, state):
        p_c = self.phi_control.update(cmd.item(0), state.phi)
        q_c = self.theta_control.update(cmd.item(1), state.theta)
        r_c = self.psi_control.update(cmd.item(2), state.psi)
        return np.array([[p_c],[q_c],[r_c]])

    def sat(self, val, low, high):
        if val < low:
            val = low
        elif val > high:
            val = high
        return val

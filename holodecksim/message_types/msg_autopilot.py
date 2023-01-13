"""
msg_autopilot
    - messages type for input to the autopilo
    - 6/25/2020 - RWB
"""
import numpy as np

class msgAutopilot:
    def __init__(self):
        self.vel = np.array([[0.], [0], [0]])
        self.psi = 0.
        self.rot = np.eye(3)  # commanded attitude (rotation matrix body to inertial)
        self.omega = np.array([[0.], [0.], [0.]])  # commanded angular rate
        self.omega_dot = np.array([[0.], [0.], [0.]])  # commanded angular acceleration

        # # from mavsim
        # self.airspeed_command = 0.0  # commanded airspeed m/s
        # self.course_command = 0.0  # commanded course angle in rad
        # self.altitude_command = 0.0  # commanded altitude in m
        # self.phi_feedforward = 0.0  # feedforward command for roll angle

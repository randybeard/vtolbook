"""
msg_autopilot
    - messages type for input to the autopilot

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/5/2019 - RWB
"""

import numpy as np

class msgAutopilot:
    def __init__(self):
        self.airspeed_command = 0.0  # commanded airspeed m/s
        self.course_command = 0.0  # commanded course angle in rad
        self.altitude_command = 0.0  # commanded altitude in m
        self.phi_feedforward = 0.0  # feedforward command for roll angle

        #commands for hover controller
        self.roll_command = 0.0
        self.pitch_command = 0.0
        self.yaw_command = 0.0
        self.pn_command = 0.0
        self.pe_command = 0.0

        # Jake's hover control
        self.pos = np.zeros((3, 1))  # commanded position in m
        self.vel = np.zeros((3, 1))  # commanded velocity in m/s
        self.accel = np.zeros((3, 1))  # commanded acceleration in m/s/s
        self.heading = 0.0  # commanded heading in rad

    def __str__(self):
        return f'airspeed_command: {self.airspeed_command}\n' + \
            f'course_command: {self.course_command}\n' + \
            f'altitude_command: {self.altitude_command}\n' + \
            f'phi_feedforward: {self.phi_feedforward}\n' + \
            f'roll_command: {self.roll_command}\n' + \
            f'pitch_command: {self.pitch_command}\n' + \
            f'yaw_command: {self.yaw_command}\n' + \
            f'pn_command: {self.pn_command}\n' + \
            f'pe_command: {self.pe_command}\n' + \
            f'pos: {self.pos.T}.T\n' + \
            f'vel: {self.vel.T}.T\n' + \
            f'accel: {self.accel.T}.T\n' + \
            f'heading: {self.heading}'
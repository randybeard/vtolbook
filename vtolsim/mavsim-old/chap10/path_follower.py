import numpy as np
from math import sin, cos, atan, atan2
import sys

sys.path.append('..')
from message_types.msg_autopilot import msgAutopilot
from tools.wrap import wrap

class pathFollower:
    def __init__(self):
        self.chi_inf = np.radians(50)  # approach angle for large distance from straight-line path
        self.k_path = 0.05  # proportional gain for straight-line path following
        self.k_orbit = 10.0  # proportional gain for orbit following
        self.gravity = 9.8
        self.autopilot_commands = msgAutopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.type=='line':
            self._follow_straight_line(path, state)
        elif path.type=='orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        self.autopilot_commands.airspeed_command = path.airspeed
        chi_q = atan2(path.line_direction.item(1),
                      path.line_direction.item(0))
        chi_q = wrap(chi_q, state.chi)
        path_error = -sin(chi_q) * (state.pn - path.line_origin.item(0)) \
                     + cos(chi_q) * (state.pe - path.line_origin.item(1))
        # course command
        self.autopilot_commands.course_command \
            = chi_q - self.chi_inf * (2 / np.pi) * np.arctan(self.k_path * path_error)
        # altitude command
        self.autopilot_commands.altitude_command \
            = -path.line_origin.item(2) \
              - np.sqrt((path.line_origin.item(0) - state.pn)**2
                        + (path.line_origin.item(1) - state.pe)**2) * path.line_direction.item(2) \
              / np.sqrt(path.line_direction.item(0)**2 + path.line_direction.item(1)**2)
        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0.0

    def _follow_orbit(self, path, state):
        if path.orbit_direction == 'CW':
            direction = 1.0
        else:
            direction = -1.0
        # airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed
        # distance from orbit center
        d = np.sqrt((state.pn - path.orbit_center.item(0))**2
                    + (state.pe - path.orbit_center.item(1))**2)
        # compute wrapped version of angular position on orbit
        varphi = np.arctan2(state.pe - path.orbit_center.item(1),
                            state.pn - path.orbit_center.item(0))
        varphi = wrap(varphi, state.chi)
        # compute normalized orbit error
        orbit_error = (d - path.orbit_radius) / path.orbit_radius
        # course command
        self.autopilot_commands.course_command \
            = varphi + direction * (np.pi/2.0 + np.arctan(self.k_orbit * orbit_error))
        # altitude command
        self.autopilot_commands.altitude_command = -path.orbit_center.item(2)
        # roll feedforward command
        if orbit_error < 0.25:
            self.autopilot_commands.phi_feedforward \
                = direction * np.arctan(path.airspeed**2 / self.gravity / path.orbit_radius)
        else:
            self.autopilot_commands.phi_feedforward = 0.0




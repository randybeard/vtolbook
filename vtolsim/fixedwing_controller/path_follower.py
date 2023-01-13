import numpy as np
from math import sin, cos
import sys

sys.path.append('..')
from message_types.msg_autopilot import msgAutopilot
from tools.wrap import wrap
import parameters.planner_parameters as PLANNER


class PathFollower:
    def __init__(self):
        self.chi_inf = np.pi/2 # approach angle for large distance from straight-line path
        self.k_path =  0.02  # proportional gain for straight-line path following
        self.k_orbit = 10.0  # proportional gain for orbit following
        self.gravity = 9.8
        self.lambda_o = .5
        self.autopilot_commands = msgAutopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):

        q = path.line_direction
        r = path.line_origin

        self.autopilot_commands.airspeed_command = path.airspeed

        # course command
        chi_q = np.arctan2(path.line_direction.item(1), path.line_direction.item(0))
        chi_q = wrap(chi_q, state.chi)
        # R_i2p = np.array([[np.cos(chi_q), np.sin(chi_q), 0],
        #                     [-np.sin(chi_q), np.cos(chi_q), 0],
        #                     [0, 0, 1]])
        # ep = R_i2p @ (state.position - path.line_origin)
        # epy = ep.item(1)
        epy = -np.sin(chi_q) * (state.pn - path.line_origin.item(0)) \
            + np.cos(chi_q) * (state.pe - path.line_origin.item(1))
        self.autopilot_commands.course_command = chi_q \
            - self.chi_inf * (2 / np.pi) * np.arctan(self.k_path*epy)

        # altitude command
        k_i = np.array([[0, 0, 1]]).T
        # np.cross(q.reshape(3,), k_i.reshape((3,)))
        n = (np.cross(q.reshape(3,), k_i.reshape((3,))) \
            / np.linalg.norm(np.cross(q.reshape(3,), k_i.reshape((3,))))).reshape((3,1))
        # n = np.cross(k_i.reshape(3,), q.reshape(3,)) \
        #     / np.linalg.norm(np.cross(k_i.reshape(3,), q.reshape(3,)))
        e_i_p = state.position - r
        s_i = e_i_p - (e_i_p.T @ n).item(0) * n
        print(n)
        self.autopilot_commands.altitude_command = -r.item(2) \
            - np.sqrt(s_i.item(0)**2 + s_i.item(1)**2) \
            * q.item(2) / (np.sqrt(q.item(0)**2 + q.item(1)**2))

        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0.0 #state.phi

    def _follow_orbit(self, path, state):
        c = path.orbit_center
        
        if path.orbit_direction == 'CW':
            direction = 1.0
        else:
            direction = -1.0
        # airspeed command
        self.autopilot_commands.airspeed_command = PLANNER.Va0
        # distance from orbit center
        d = np.sqrt((state.north-c.item(0))**2 + (state.east-c.item(1))**2)
        # compute wrapped version of angular position on orbit
        varphi = np.arctan2(state.east - c.item(1), state.north - c.item(0))
        while varphi - state.chi < -np.pi:
            varphi += 2*np.pi
        while varphi - state.chi > np.pi:
            varphi -= 2*np.pi
        # compute normalized orbit error
        orbit_error = 0
        # course command
        chi_o = varphi + direction*np.pi/2
        self.autopilot_commands.course_command = chi_o + \
            direction*np.arctan(self.k_orbit*((d-path.orbit_radius)/path.orbit_radius))
        # altitude command
        self.autopilot_commands.altitude_command = -path.orbit_center.item(2)
        # roll feedforward command
        if orbit_error < 10:
            self.autopilot_commands.phi_feedforward = 0
        else:
            self.autopilot_commands.phi_feedforward = direction * \
                np.arctan(((state.wn*np.cos(state.chi)) + np.sqrt(
                    state.Va**2 - (state.wn*np.sin(state.chi) - state.we*np.cos(state.chi))**2 \
                    -state.wd**3))**2 \
                / self.gravity*path.orbit_radius*np.sqrt(
                    (state.Va**2 - (state.wn*np.sin(state.chi) - state.we*np.cos(state.chi))**2 \
                    - state.wd**2) / (state.Va**2 - state.wd**2)
                ))




"""
autopilot
        6/24/2020 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.control_parameters as CTRL
from tools.transfer_function import transferFunction
from tools.wrap import wrap
from message_types.msg_state import msgState
from message_types.msg_delta import msgDelta
from chap4.controllers import ctrlAngularRate, ctrlAttitudeSimple, ctrlVelocity


class autopilot:
    def __init__(self, ts_control):
        # instantiate controllers
        self.ctrl_angular_rate = ctrlAngularRate(Kp=CTRL.Kp_angular_rate, J=CTRL.J)
        self.ctrl_attitude = ctrlAttitudeSimple(Kp=CTRL.Kp_attitude_simple, Ts=ts_control)
        self.ctrl_velocity = ctrlVelocity(Kd=CTRL.Kd_velocity, Ts=ts_control)
        self.commanded_state = msgState()

    def update(self, cmd, state):
        T_d, rot_d, omega_d = self.ctrl_velocity.update(state.vel, state.rot, cmd.vel, cmd.psi)
        omega_cmd, omega_dot_cmd = self.ctrl_attitude.update(state.rot, state.omega, rot_d, omega_d)
        tau = self.ctrl_angular_rate.update(state.omega, omega_cmd, omega_dot_cmd)
        #F = CTRL.mass * CTRL.gravity + 10
        F = CTRL.mass * T_d


        # # lateral autopilot
        # chi_c = wrap(cmd.course_command, state.chi)
        # phi_c = self.saturate(
        #     cmd.phi_feedforward + self.course_from_roll.update(chi_c, state.chi),
        #     -np.radians(30), np.radians(30))
        # delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p)
        # delta_r = self.yaw_damper.update(state.r)
        #
        # # longitudinal autopilot
        # # saturate the altitude command
        # h_c = self.saturate(cmd.altitude_command, state.h - CTRL.altitude_zone, state.h + CTRL.altitude_zone)
        # theta_c = self.altitude_from_pitch.update(h_c, state.h)
        # delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q)
        # delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va)
        # delta_t = self.saturate(delta_t, 0.0, 1.0)

        # construct output and commanded states
        delta = msgDelta(force=F, torque=tau, gimbal_input=np.zeros((3,1)))
        # self.commanded_state.h = cmd.altitude_command
        # self.commanded_state.Va = cmd.airspeed_command
        # self.commanded_state.phi = phi_c
        # self.commanded_state.theta = theta_c
        # self.commanded_state.chi = cmd.course_command
        self.commanded_state.omega = cmd.omega
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output


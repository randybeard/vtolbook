"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.fw_control_parameters as AP
from tools.transfer_function import transferFunction
from tools.wrap import wrap
from tools.rotations import Euler2Quaternion
from fixedwing_controller.pid_control import pidControl, piControl, pdControlWithRate
from message_types.msg_state import msgState
from message_types.msg_controls import msgControls


class autopilot:
    def __init__(self, ts_control):
        # instantiate lateral controllers
        self.roll_from_aileron = pdControlWithRate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = piControl(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        # self.sideslip_from_rudder = piControl(
        #                 kp=AP.sideslip_kp,
        #                 ki=AP.sideslip_ki,
        #                 Ts=ts_control,
        #                 limit=np.radians(45))
        # self.yaw_damper = transferFunction(
        #                 num=np.array([[AP.yaw_damper_kp, 0]]),
        #                 den=np.array([[1, 1/AP.yaw_damper_tau_r]]),
        #                 Ts=ts_control)

        # instantiate lateral controllers
        self.pitch_from_elevator = pdControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        self.altitude_from_pitch = piControl(
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.airspeed_from_throttle = piControl(
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0)
        self.commanded_state = msgState()

    def update(self, cmd, state):

        # lateral autopilot
        chi_c = wrap(cmd.course_command, state.chi)
        phi_c = self.saturate(
            cmd.phi_feedforward + self.course_from_roll.update(chi_c, state.chi),
            -np.radians(30), np.radians(30))
        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p)
        # delta_r = self.yaw_damper.update(state.r)

        # longitudinal autopilot
        # saturate the altitude command
        h_c = self.saturate(cmd.altitude_command, state.h - AP.altitude_zone, state.h + AP.altitude_zone)
        theta_c = self.altitude_from_pitch.update(h_c, state.h)
        delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q)
        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va)
        delta_t = self.saturate(delta_t, 0.0, 1.0)

        # construct output and commanded states
        delta = msgControls()
        delta.elevon_right = 0.5*delta_e - 0.5*delta_a
        delta.elevon_left = 0.5*delta_e + 0.5*delta_a
        delta.throttle_rear = 0.0
        delta.throttle_right = delta_t
        delta.throttle_left = delta_t
        delta.servo_right = np.radians(0)
        delta.servo_left = np.radians(0)

        self.commanded_state.h = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output

    def desired_state(self):

        desired_state = np.concatenate([
            np.zeros((3,1)), # position
            np.array([[self.commanded_state.Va]]), np.zeros((2,1)),
            Euler2Quaternion(
                self.commanded_state.phi, self.commanded_state.theta, self.commanded_state.chi).reshape((-1,1)),
            np.zeros((3,1))])
        return desired_state
        #     pd_i,
        #     vd_b,
        #     Rotation2Quaternion(R_d).reshape(-1),
        #     omega_c]).reshape((-1, 1))

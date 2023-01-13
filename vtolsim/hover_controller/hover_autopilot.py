"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.hover_control_parameters as AP
import parameters.convergence_parameters as VTOL
from tools.transfer_function import transferFunction
from tools.wrap import wrap
from hover_controller.pid_control import pidControl, piControl, pdControlWithRate
from message_types.msg_state import msgState
from message_types.msg_controls import msgControls
from hover_controller.compute_delta import compute_delta
import math


class hoverController:
    def __init__(self, ts_control):
        self.altitude_zone = AP.altitude_zone
        self.roll_from_moment = pidControl(
                        kp=AP.roll_kp,
                        ki=AP.roll_ki,
                        kd=AP.roll_kd,
                        Ts=ts_control,
                        limit=np.inf)
        self.pitch_from_moment = pidControl(
                        kp=AP.pitch_kp,
                        ki=AP.pitch_ki,
                        kd=AP.pitch_kd,
                        Ts=ts_control,
                        limit=np.inf)
        self.altitude_from_uz = pidControl(
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        kd=AP.altitude_kd,
                        Ts=ts_control,
                        limit=np.inf)
        self.altitude_from_uz.update(-VTOL.pd0, -VTOL.pd0, reset_flag=True)
        self.yaw_from_moment = pidControl(
                        kp=AP.yaw_kp,
                        ki=AP.yaw_ki,
                        kd=AP.yaw_kd,
                        Ts=ts_control,
                        limit=np.inf,
                        ispsi=True)
        self.pn_from_ux = pidControl(
                        kp=AP.pn_kp,
                        ki=AP.pn_ki,
                        kd=AP.pn_kd,
                        Ts=ts_control,
                        limit=np.inf)
        self.pe_from_uy = pidControl(
                        kp=AP.pe_kp,
                        ki=AP.pe_ki,
                        kd=AP.pe_kd,
                        Ts=ts_control,
                        limit=np.inf)
        self.commanded_state = msgState()

    def update(self, cmd, state):

        # saturate the altitude command
        h_c = self.saturate(cmd.altitude_command, state.h - self.altitude_zone, state.h + self.altitude_zone)
        ux = self.pn_from_ux.update(cmd.pn_command, state.pn)
        uy = self.pe_from_uy.update(cmd.pe_command, state.pe)
        uz = self.altitude_from_uz.update(h_c, state.h)

        phi_c = math.atan(uy*np.cos(state.theta)/(VTOL.gravity-uz))
        theta_c = math.atan(ux/(uz-VTOL.gravity))
        # phi_c = cmd.roll_command
        # theta_c = cmd.pitch_command
        F = VTOL.mass*(VTOL.gravity-uz)/(np.cos(state.phi)*np.cos(state.theta)) / (2.0*VTOL.mass*VTOL.gravity)

        phi_c = self.saturate(phi_c, -30*np.pi/180, 30*np.pi/180);
        theta_c = self.saturate(theta_c, -20*np.pi/180, 20*np.pi/180);

        tau_phi = self.roll_from_moment.update_with_rate(phi_c, state.phi, state.p)
        tau_theta = self.pitch_from_moment.update_with_rate(theta_c, state.theta, state.q)
        tau_psi = self.yaw_from_moment.update_with_rate(cmd.yaw_command, state.psi, state.r)

        # delta_command = compute_delta(np.array([[0.0],[-F],[tau_phi],[tau_theta],[tau_psi]]))

        #testing ROSFlight mixer strategy
        delta_command = np.zeros((5,1))
        delta_command[0] = 1.08*F - 1.333*tau_theta
        delta_command[1] = 0.92*F - tau_phi + 0.667*tau_theta
        delta_command[2] = 0.92*F + tau_phi + 0.667*tau_theta
        delta_command[3] = -tau_psi + np.pi/2.0
        delta_command[4] = tau_psi + np.pi/2.0

        for i in range(3):
            if delta_command[i] < 0.0:
                delta_command[i] = 0.0
            elif delta_command[i] > 1.0:
                delta_command[i] = 1.0

        for i in range(3,5):
            if delta_command[i] < np.radians(-15.0):
                delta_command[i] = np.radians(-15.0)
            elif delta_command[i] > np.radians(115.0):
                delta_command[i] = np.radians(115.0)

        # print(delta_command)
        # adfa
        # construct output and commanded states
        delta = msgControls()
        delta.elevon_right = 0.0
        delta.elevon_left = 0.0
        delta.throttle_rear = delta_command.item(0)
        delta.throttle_right = delta_command.item(1)
        delta.throttle_left = delta_command.item(2)
        delta.servo_right = delta_command.item(3)
        delta.servo_left = delta_command.item(4)

        self.commanded_state.h = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.yaw_command
        self.commanded_state.pn = cmd.pn_command
        self.commanded_state.pe = cmd.pe_command
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output

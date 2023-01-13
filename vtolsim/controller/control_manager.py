#!/usr/bin/python3

import sys
sys.path.append('..')

from geometric_control.geometric_controller import GeometricController as GC
from fixedwing_controller.autopilot import autopilot
from fixedwing_controller.path_follower import PathFollower
import parameters.simulation_parameters as SIM
import parameters.convergence_parameters as VTOL
from message_types.msg_state import msgState
from tools.rotations import *
import controller.control_switch as control_switch

CTRL_GEOMETRIC = 0
CTRL_PATH_FOLLOWER = 1
CTRL_QUADROTOR = 2

SWITCH_RAPID_LANDING = 0

class ControlManager():

    def __init__(self, controller_type=CTRL_GEOMETRIC, switch_type=SWITCH_RAPID_LANDING, 
        Ts=SIM.ts_control, data=None):

        # set up controller
        self.controller_type = controller_type
        if self.controller_type == CTRL_GEOMETRIC:
            self.controller = GC(Ts, theta_0=VTOL.theta0)
        elif self.controller_type == CTRL_PATH_FOLLOWER:
            self.controller = autopilot(Ts)
            self.path_follower = PathFollower()

        # set up control switcher
        self.switch_type = switch_type
        if self.switch_type == SWITCH_RAPID_LANDING:
            assert data != None, 'ERROR: Need to include transition height'
            self.control_switch = control_switch.SwitchRapidLanding(transition_z=data[0])
            
        
    def update(self, data):
        if self.controller_type == CTRL_GEOMETRIC:
            delta = None
        elif self.controller_type == CTRL_PATH_FOLLOWER:
            path = data[0]
            state = self.state_array_2_state_msg(data[1])
            cmd = self.path_follower.update(path, state)
            delta, _ = self.controller.update(cmd, state)
        return delta

    def desired_state(self):
        return self.controller.desired_state()

    def state_array_2_state_msg(self, state):
        state_msg = msgState()
        state_msg.pn = state.item(0)
        state_msg.pe = state.item(1)
        state_msg.h = -state.item(2)
        state_msg.Va = np.sqrt(state.item(3)**2 + state.item(4)**2 + state.item(5)**2)
        state_msg.Vg = state_msg.Va
        state_msg.u = state.item(3)
        state_msg.v = state.item(4)
        state_msg.w = state.item(5)
        state_msg.phi, state_msg.theta, state_msg.psi = Quaternion2Euler(state[6:10,:])
        state_msg.chi = state_msg.psi
        state_msg.gamma = state_msg.theta
        state_msg.p = state.item(10)
        state_msg.q = state.item(11)
        state_msg.r = state.item(12)
        return state_msg

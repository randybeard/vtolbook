"""
    convert messages to/from numpy arrays
"""
import sys
sys.path.append('..')
import numpy as np
from message_types.msg_controls import msgControls
from message_types.msg_state import msgState

def np2msg_controls(delta):
        msg = msgControls()
        msg.elevon_right = delta.item(0)    # right elevon angle in radians
        msg.elevon_left = delta.item(1)     # left elevon in radians
        msg.throttle_rear = delta.item(2)   # commanded throttle for rear rotor
        msg.throttle_right = delta.item(3)  # commanded throttle for right rotor
        msg.throttle_left = delta.item(4)   # commanded throttle for left rotor
        msg.servo_right = delta.item(5)     # commanded right servo angle in radians
        msg.servo_left = delta.item(6)      # commanded left servo angle in radians
        return msg

def msg_controls2np(msg):
    return np.array([[ msg.elevon_right, msg.elevon_left, \
        msg.throttle_rear, msg.throttle_right, msg.throttle_left, \
        msg.servo_right, msg.servo_left ]]).T

def msg_state2np(msg):
        return np.array([[msg.pn],  # [0]  north position
                        [msg.pe],   # [1]  east position
                        [-msg.h],   # [2]  down position
                        [msg.u],    # [3]  velocity along body x-axis
                        [msg.v],    # [4]  velocity along body y-axis
                        [msg.w],    # [5]  velocity along body z-axis
                        [msg.phi],  # [6]  pitch angle
                        [msg.theta],# [7]  roll angle
                        [msg.psi],  # [8]  yaw angle
                        [msg.p],   # [9]  roll rate
                        [msg.q],   # [10]  pitch rate
                        [msg.r],   # [11]  yaw rate
                        [msg.right_rotor],  # [12] pitch angle of right motor
                        [msg.left_rotor],   # [13] pitch angle of left motor
                        ])


def np2msg_state(x):
    msg = msgState()
    msg.pn   = x.item(0) # [0]  north position
    msg.pe   = x.item(1) # [1]  east position
    msg.h    = -x.item(2) # [2]  down position
    msg.u    = x.item(3) # [3]  velocity along body x-axis
    msg.v    = x.item(4) # [4]  velocity along body y-axis
    msg.w    = x.item(5) # [5]  velocity along body z-axis
    msg.phi  = x.item(6) # [6]  pitch angle
    msg.theta= x.item(7) # [7]  roll angle
    msg.psi  = x.item(8) # [8]  yaw angle
    msg.p    = x.item(9) # [9]  roll rate
    msg.q    = x.item(10) # [10]  pitch rate
    msg.r    = x.item(11) # [11]  yaw rate
    msg.right_rotor = x.item(12) # [12] pitch angle of right motor
    msg.left_rotor = x.item(13)   # [13] pitch angle of left motor
    
    return msg




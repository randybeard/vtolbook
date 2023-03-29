"""
msgState 
    - messages type for vtol state
    - Update history:  
        3/19/2020 - RWB
        3/23/2022 - RWB
"""
import numpy as np
from tools.rotations import rotation_to_quaternion, quaternion_to_rotation


class MsgState:
    def __init__(self,
                 pos=np.array([[0.], [0.], [0.]]),
                 vel=np.array([[0.], [0.], [0.]]),
                 rot=np.eye(3),
                 omega=np.array([[0.], [0.], [0.]]),
                 bias=np.array([[0.], [0.], [0.]]),
                 rotors=np.array([[0.], [0.]])
                 ):
        self.pos = pos  # position in inertial frame
        self.vel = vel  # velocity in inertial frame
        self.rot = rot  # rotation from body to to inertial
        self.omega = omega  # angular velocity in body frame
        self.bias = bias  # gyro biases
        self.rotors = rotors  # front rotor angles

    def to_array(self):
        quat = rotation_to_quaternion(self.rot)
        x = np.array([[self.pos.item(0)],
                      [self.pos.item(1)],
                      [self.pos.item(2)],
                      [self.vel.item(0)],
                      [self.vel.item(1)],
                      [self.vel.item(2)],
                      [quat.item(0)],
                      [quat.item(1)],
                      [quat.item(2)],
                      [quat.item(3)],
                      [self.omega.item(0)],
                      [self.omega.item(1)],
                      [self.omega.item(2)],
                      [self.bias.item(0)],
                      [self.bias.item(1)],
                      [self.bias.item(2)],
                      [self.rotors.item(0)],
                      [self.rotors.item(1)]
                      ])
        return x

    def from_array(self, x):
        self.pos = x[0][0:3]
        self.vel = x[0][3:6]
        quat = x[0][6:10]
        quat = quat / np.linalg.norm(quat)
        self.rot = quaternion_to_rotation(quat)
        self.omega = x[10:13]
        self.bias = x[13:16]
        self.rotors = x[16:18]

    def print(self):
        print('position=', self.pos,
              'velocity=', self.vel,
              'rotation=', self.rot,
              'angular velocity=', self.omega,
              'gyro bias=', self.bias,
              'rotor angles=', self.rotors)
"""
msg_delta
    - messages type for input to the multirotor
    - Last update:
        3/19/2020 - RWB
        6/23/2020 - RWB
"""
import numpy as np

class msgDelta:
    def __init__(self,
                 force=0.0,
                 torque=np.array([[0.], [0.], [0.]]),
                 gimbal_input=np.array([[0.], [0.], [0.]])
                 ):
        self.force = force # total force in -z direction
        self.torque = torque  # torque in body frame
        self.gimbal_input = gimbal_input  # angular velocity command for gimbal

    def to_array(self):
        return np.array([[self.force],
                         [self.torque.item(0)],
                         [self.torque.item(1)],
                         [self.torque.item(2)],
                         [self.gimbal_input.item(0)],
                         [self.gimbal_input.item(1)],
                         self.gimbal_input.item(2)])

    def from_array(self, u):
        self.force = u.item(0)
        self.torque[0][0] = u.item(1)
        self.torque[1][0] = u.item(2)
        self.torque[2][0] = u.item(3)
        self.gimbal_input[0][0] = u.item(4)
        self.gimbal_input[1][0] = u.item(5)
        self.gimbal_input[2][0] = u.item(6)

    def print(self):
        print('force=', self.force,
              'torque=', self.torque,
              'gimbal_input=', self.gimbal_input)


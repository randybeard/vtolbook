"""
msg_motor_commands
    - messages type for individual motor inputs into the system
    - Last update:
        3/19/2020 - RWB
"""
import numpy as np

class MsgDelta:
    def __init__(self,
                 forces=np.array([[0.], [0.], [0.], [0.]])
                 ):
        self.forces = forces # forces from each motor (the torques are porportional to these forces)

    def to_array(self):
        return np.array([[self.forces.item(0)],
                         [self.forces.item(1)],
                         [self.forces.item(2)],
                         [self.forces.item(3)]])

    def from_array(self, u):
        self.forces[0][0] = u.item(0)
        self.forces[1][0] = u.item(1)
        self.forces[2][0] = u.item(2)
        self.forces[3][0] = u.item(3)

    def print(self):
        print('force=', self.force,
              'torque=', self.torque)

import numpy as np

class MsgAutopilot:
    def __init__(self):
        self.pos = np.zeros((3,1))
        self.vel = np.zeros((3,1))
        self.accel = np.zeros((3,1))
        self.heading = 0.0
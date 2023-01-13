"""
msg_autopilot
    - messages type for input to the autopilot
    
part of quadsim_python
    - Last update:
        3/3/2022 - RWB
"""
import numpy as np

class MsgAutopilot:
    def __init__(self):
        self.pos = np.zeros((3, 1))  # commanded position in m
        self.vel = np.zeros((3, 1))  # commanded velocity in m/s
        self.accel = np.zeros((3, 1))  # commanded acceleration in m/s/s
        self.heading = 0.0  # commanded heading in rad

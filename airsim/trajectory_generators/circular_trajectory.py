import numpy as np
from message_types.msg_autopilot import MsgAutopilot

class TrajectoryGenerator:
    def __init__(self, Ts):
        self.Ts = Ts
        self.time = 0

    def update(self):
        msg = MsgAutopilot()
        
        planar_amplitude = 5
        planar_frequency = 1./5*2*np.pi

        vertical_amplitude = 0.5
        vertical_frequency = planar_amplitude/20

        msg.pos = np.array([[planar_amplitude*np.sin(planar_frequency*self.time), planar_amplitude*np.cos(planar_frequency*self.time), -5 + vertical_amplitude*np.sin(vertical_frequency*self.time)]]).T
        msg.vel = np.array([[planar_amplitude*planar_frequency*np.cos(planar_frequency*self.time), -planar_amplitude*planar_frequency*np.sin(planar_frequency*self.time), vertical_amplitude*vertical_frequency*np.cos(vertical_frequency*self.time)]]).T
        msg.accel = np.array([[-planar_amplitude*(planar_frequency**2)*np.sin(planar_frequency*self.time), -planar_amplitude*(planar_frequency**2)*np.cos(planar_frequency*self.time), -vertical_amplitude*(vertical_frequency**2)*np.sin(vertical_frequency*self.time)]]).T

        self.time += self.Ts
        return msg
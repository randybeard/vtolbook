import numpy as np 
import param as P

class OwnshipDynamics:
    def __init__(self):
        self.pos = P.ownship_p0
        self.vel = P.ownship_v0
        # simulation time step
        self.Ts = P.Ts

    def update(self, u):
        self.pos = self.pos + self.Ts * self.vel
        self.vel = self.vel + self.Ts * u

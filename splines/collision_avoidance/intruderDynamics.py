import numpy as np 
import param as P

class IntruderDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.pos = P.intruder_p0
        self.vel = P.intruder_v0
        self.covar = P.intruder_covar0
        # simulation time step
        self.Ts = P.Ts

    def update(self):
        self.pos = self.pos + self.Ts * self.vel
        self.vel = self.vel + self.Ts * 0.01 * np.random.randn(3, 1)

"""
dirty derivative
"""
import numpy as np
import matplotlib.pyplot as plt

class DirtyDerivative:
    def __init__(self, Ts, tau):
        self.Ts = Ts
        self.tau = tau
        self.a1 = (2.0 * tau - Ts) / (2.0 * tau + Ts)
        self.a2 = 2.0 / (2.0 * tau+ Ts)
        self.initialized = False

    def update(self, z):
        if self.initialized is False:
            self.z_dot = 0 * z
            self.z_delay_1 = z
            self.initialized = True
        else:
            self.z_dot = self.a1 * self.z_dot \
                         + self.a2 * (z - self.z_delay_1)
            self.z_delay_1 = z
        return self.z_dot
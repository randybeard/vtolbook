"""
pid_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        7/13/2021 - SMN
"""
import sys
import numpy as np
sys.path.append('..')

class pidControl:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.01, sigma=0.05, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.limit = limit
        self.integrator = 0.0
        self.error_delay_1 = 0.0
        self.error_dot_delay_1 = 0.0
        self.y_delay_1 = 0.0
        self.y_dot_delay_1 = 0.0
        self.sigma = sigma

    def update(self, y_ref, y, Ts=None, reset_flag=False):
        if Ts is None:
            Ts = self.Ts
        if reset_flag == True:
            self.integrator = 0.0
            self.error_delay_1 = 0.0
            self.y_dot = 0.0
            self.y_delay_1 = 0.0
            self.y_dot_delay_1 = 0.0
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator \
                          + (Ts/2) * (error + self.error_delay_1)
        # update the differentiator
        a1 = (2.0 * self.sigma - Ts) / (2.0 * self.sigma + Ts)
        a2 = 2.0 / (2.0 * self.sigma + Ts)
        y_dot = a1 * self.y_dot_delay_1 \
                         + a2 * (y - self.y_delay_1)
        # PID control
        u = self.kp * error \
            + self.ki * self.integrator \
            - self.kd * y_dot
        # saturate PID control at limit
        u_sat = self._saturate(u)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator \
                              + (Ts / self.ki) * (u_sat - u)
        # update the delayed variables
        self.y_delay_1 = y
        self.y_dot_delay_1 = y_dot
        self.error_delay_1 = error
        return u_sat

    def update_with_rate(self, y_ref, y, ydot, Ts=None, reset_flag=False):
        if Ts is None:
            Ts = self.Ts
        if reset_flag == True:
            self.integrator = 0.0
            self.error_delay_1 = 0.0
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator \
                          + (Ts/2) * (error + self.error_delay_1)
        # PID control
        u = self.kp * error \
            + self.ki * self.integrator \
            - self.kd * ydot
        # saturate PID control at limit
        u_sat = self._saturate(u)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator \
                              + (Ts / self.ki) * (u_sat - u)
        self.error_delay_1 = error
        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat

class piControl:
    def __init__(self, kp=0.0, ki=0.0, Ts=0.01, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.Ts = Ts
        self.limit = limit
        self.integrator = 0.0
        self.error_delay_1 = 0.0

    def update(self, y_ref, y, Ts=None):
        if Ts is None:
            Ts = self.Ts
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator \
                          + (Ts/2) * (error + self.error_delay_1)
        # PI control
        u = self.kp * error \
            + self.ki * self.integrator
        # saturate PI control at limit
        u_sat = self._saturate(u)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator \
                              + (Ts / self.ki) * (u_sat - u)
        # update the delayed variables
        self.error_delay_1 = error
        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat

class pdControlWithRate:
    # PD control with rate information
    # u = kp*(yref-y) - kd*ydot
    def __init__(self, kp=0.0, kd=0.0, limit=1.0):
        self.kp = kp
        self.kd = kd
        self.limit = limit

    def update(self, y_ref, y, ydot):
        u = self.kp * (y_ref - y)  - self.kd * ydot
        # saturate PID control at limit
        u_sat = self._saturate(u)
        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat

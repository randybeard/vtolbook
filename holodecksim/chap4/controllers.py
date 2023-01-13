"""
controllers
        6/24/2020 - RWB
        6/29/2020 - RWB
"""
import numpy as np
from tools.rotations import hat, vee, skew

class ctrlAngularRate:
    def __init__(self, Kp=np.eye(3), J=np.eye(3)):
        self.Kp = Kp
        self.J = J

    def update(self, omega, omega_d, omega_dot_d):
        tau = hat(omega) @ self.J @ omega \
              + self.J @ omega_dot_d \
              - self.J @ self.Kp @ (omega - omega_d)
        return tau


class ctrlAttitudeSimple:
    def __init__(self, Kp=np.eye(3), Ts=0.01, sigma=0.05):
        self.Kp = Kp
        self.Ts = Ts
        self.Rd_dot = differentiator(size=(3, 3), Ts=Ts, sigma=sigma)
        self.omega_d_dot = differentiator(size=(3, 1), Ts=Ts, sigma=sigma)

    def update(self, R, omega, Rd, omega_d=np.nan*np.zeros((3,1))):
        # if omega_d is not passed into the update function, then numerically differentiate Rd to get omega_d
        if omega_d.item(0) == np.nan:
            Rd_dot = self.Rd_dot.update(Rd)
            omega_d = vee(Rd.T @ Rd_dot)
        omega_d_dot = self.omega_d_dot.update(omega_d)
        omega_cmd = R.T @ Rd @ omega_d + self.Kp @ vee(skew(R.T @ Rd))
        omega_cmd_dot = -hat(omega) @ R.T @ R @ omega_d \
                        + R.T @ Rd @ omega_d_dot \
                        + self.Kp @ vee(skew(R.T @ Rd @ hat(omega_d) - hat(omega) @ R.T @ Rd))
        return omega_cmd, omega_cmd_dot


class ctrlVelocity:
    def __init__(self, Kd=np.eye(3), gravity=9.8, Ts=0.01, sigma=0.05):
        self.Kd = Kd
        self.gravity = gravity
        self.Ts = Ts
        self.rot_d_dot = differentiator(size=(3, 3), Ts=Ts, sigma=sigma)

    def update(self, vel, rot, vel_d, psi_d):
        e3 = np.array([[0.], [0.], [1.]])
        tmp = self.gravity * e3 - self.Kd @ (vel_d - vel)
        T_d = tmp.T @ rot @ e3
        rd3 = tmp / np.linalg.norm(tmp)
        s_psi_d = np.array([[np.cos(psi_d)], [np.sin(psi_d)], [0.]])
        tmp2 = hat(rd3) @ s_psi_d
        rd2 = tmp2 / np.linalg.norm(tmp2)
        rd1 = hat(rd2) @ rd3
        rot_d = np.concatenate((rd1, rd2, rd3), axis=1)
        rot_d_dot = self.rot_d_dot.update(rot_d)
        omega_d = vee(rot_d.T @ rot_d_dot)
        return T_d, rot_d, omega_d,


class differentiator:
    #  implement dirty derivative
    def __init__(self, size=(1, 1), Ts=0.01, sigma=0.05):
        self.size = size
        self.a1 = (2.0 * sigma - Ts) / (2.0 * sigma + Ts)
        self.a2 = 2.0 / (2.0 * sigma + Ts)
        self.y_dot = np.zeros(size)
        self.y_delay_1 = np.zeros(size)

    def update(self, y, reset_flag=False):
        if reset_flag is True:
            self.y_dot = np.zeros(self.size)
            self.y_delay_1 = np.zeros(self.size)
        # update the differentiator
        self.y_dot = self.a1 * self.y_dot + self.a2 * (y - self.y_delay_1)
        self.y_delay_1 = y
        if self.size == (1, 1):
            return self.y_dot.item(0)
        else:
            return self.y_dot


class integrator:
    # integrate using trapazoidal rule
    def __init__(selfself, size=1, Ts=0.01):
        self.size = size
        self.Ts = Ts
        self.y_int = np.zeros((size, 1))
        self.y_delay_1 = np.zeros((size, 1))

    def update(self, y, reset_flag=False):
        if reset_flag is True:
            self.y_int = np.zeros((self.size, 1))
            self.y_delay_1 = np.zeros((self.size, 1))
        self.y_int = self.y_int + (self.Ts/2) * (y + self.y_delay_1)
        self.y_delay_1 = y
        if self.size == 1:
            return self.y_int.item(0)
        else:
            return self.y_int


class ctrlPID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.01, sigma=0.05, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.limit = limit
        self.error_dot = differentiator(size=1, Ts=Ts, sigma=sigma)
        self.error_int = integrator(size=1, Ts=Ts)

    def update(self, y_ref, y, reset_flag=False):
        # compute the error
        error = y_ref - y
        error_int = self.error_int.update(error, reset_flag)
        error_dot = self.error_dot.update(error, reset_flag)
        # PID control
        u = self.kp * error \
            + self.ki * error_int \
            + self.kd * error_dot
        # saturate PID control at limit
        u_sat = self._saturate(u)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator \
                              + (self.Ts / self.ki) * (u_sat - u)
        # update the delayed variables
        self.error_delay_1 = error
        return u_sat

    def update_with_rate(self, y_ref, y, ydot, reset_flag=False):
        if reset_flag is True:
            self.integrator = 0.0
            self.error_delay_1 = 0.0
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator \
                          + (self.Ts/2) * (error + self.error_delay_1)
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
                              + (self.Ts / self.ki) * (u_sat - u)
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


class ctrlPI:
    def __init__(self, kp=0.0, ki=0.0, Ts=0.01, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.Ts = Ts
        self.limit = limit
        self.integrator = 0.0
        self.error_delay_1 = 0.0

    def update(self, y_ref, y):

        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator \
                          + (self.Ts/2) * (error + self.error_delay_1)
        # PI control
        u = self.kp * error \
            + self.ki * self.integrator
        # saturate PI control at limit
        u_sat = self._saturate(u)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator \
                              + (self.Ts / self.ki) * (u_sat - u)
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


class ctrlPDWithRate:
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
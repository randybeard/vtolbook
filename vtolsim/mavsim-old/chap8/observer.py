"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
"""
import sys
import numpy as np
from scipy import stats
sys.path.append('..')
import parameters.control_parameters as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
from tools.rotations import Euler2Rotation
from tools.wrap import wrap

from message_types.msg_state import msgState

class observer:
    def __init__(self, ts_control):
        # initialized estimated state message
        self.estimated_state = msgState()
        # use alpha filters to low pass filter gyros and accels
        self.lpf_gyro_x = alphaFilter(alpha=0.5)
        self.lpf_gyro_y = alphaFilter(alpha=0.5)
        self.lpf_gyro_z = alphaFilter(alpha=0.5)
        self.lpf_accel_x = alphaFilter(alpha=0.5)
        self.lpf_accel_y = alphaFilter(alpha=0.5)
        self.lpf_accel_z = alphaFilter(alpha=0.5)
        # use alpha filters to low pass filter static and differential pressure
        self.lpf_static = alphaFilter(alpha=0.9)
        self.lpf_diff = alphaFilter(alpha=0.5)
        # ekf for phi and theta
        self.attitude_ekf = ekfAttitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = ekfPosition()

    def update(self, measurement):

        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) - self.estimated_state.bx
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) - self.estimated_state.by
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) - self.estimated_state.bz

        # invert sensor model to get altitude and airspeed
        static_pressure = self.lpf_static.update(measurement.static_pressure)
        diff_pressure = self.lpf_diff.update(measurement.diff_pressure)
        self.estimated_state.h = static_pressure/CTRL.rho/CTRL.gravity
        self.estimated_state.Va = np.sqrt(2 * diff_pressure / CTRL.rho)

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(measurement, self.estimated_state)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state

class alphaFilter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.alpha * self.y + (1-self.alpha) * u
        return self.y

class ekfAttitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):
        self.Q = 1e-9 * np.diag([1.0, 1.0])
        self.Q_gyro = SENSOR.gyro_sigma**2 * np.diag([1.0, 1.0, 1.0])
        self.R_accel = SENSOR.accel_sigma**2 * np.diag([1.0, 1.0, 1.0])
        self.N = 2  # number of prediction step per sample
        self.xhat = np.array([[0.0], [0.0]]) # initial state: phi, theta
        self.P = np.diag([1.0, 1.0])
        self.Ts = SIM.ts_control/self.N
        self.accel_threshold = stats.chi2.isf(q=0.01, df=3)

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        phi = x.item(0)
        theta = x.item(1)
        p = measurement.gyro_x - state.bx
        q = measurement.gyro_y - state.by
        r = measurement.gyro_z - state.bz
        G = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                      [0.0, np.cos(phi), -np.sin(phi)]])
        f_ = G @ np.array([[p], [q], [r]])
        return f_

    def h(self, x, measurement, state):
        # measurement model y
        phi = x.item(0)
        theta = x.item(1)
        p = measurement.gyro_x - state.bx
        q = measurement.gyro_y - state.by
        r = measurement.gyro_z - state.bz
        Va = np.sqrt(2 * measurement.diff_pressure / CTRL.rho)
        h_ = np.array([
            [q * Va * np.sin(theta) + CTRL.gravity * np.sin(theta)],  # x-accel
            [r * Va * np.cos(theta) - p * Va * np.sin(theta) - CTRL.gravity * np.cos(theta) * np.sin(phi)],
            # y-accel
            [-q * Va * np.cos(theta) - CTRL.gravity * np.cos(theta) * np.cos(phi)],  # z-accel
        ])
        return h_

    def propagate_model(self, measurement, state):
        # model propagation
        for i in range(0, self.N):
            phi = self.xhat.item(0)
            theta = self.xhat.item(1)
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # compute G matrix for gyro noise
            G = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                          [0.0, np.cos(phi), -np.sin(phi)]])
            # convert to discrete time models
            A_d = np.eye(2) + self.Ts * A + (self.Ts ** 2) * A @ A / 2.0
            G_d = G * self.Ts
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 * self.Q + G_d @ self.Q_gyro @ G_d.T

    def measurement_update(self, measurement, state):
        # measurement updates
        h = self.h(self.xhat, measurement, state)
        C = jacobian(self.h, self.xhat, measurement, state)
        y = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T
        S_inv = np.linalg.inv(self.R_accel + C @ self.P @ C.T)
        if (y-h).T @ S_inv @ (y-h) < self.accel_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(2) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_accel @ L.T
            self.xhat = self.xhat + L @ (y - h)

class ekfPosition:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    def __init__(self):
        self.Q = np.diag([
                    0.1,  # pn
                    0.1,  # pe
                    0.1,  # Vg
                    0.0001, # chi
                    0.1, # wn
                    0.1, # we
                    0.0001, # psi
                    ])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.R_pseudo = np.diag([
                    0.000001,  # pseudo measurement #1
                    0.000001,  # pseudo measurement #2
                    ])
        self.N = 10  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = np.array([[0.0], [0.0], [25.0], [0.0], [0.0], [0.0], [0.0]])
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999
        self.pseudo_threshold = stats.chi2.isf(q=0.01, df=2)
        #self.gps_threshold = stats.chi2.isf(q=0.01, df=4)
        self.gps_threshold = 100000 # don't gate GPS

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.pn = self.xhat.item(0)
        state.pe = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        psidot = (state.q * np.sin(state.phi) + state.r * np.cos(state.phi)) / np.cos(state.theta)
        Vgdot = ((state.Va * np.cos(psi) + wn) * (-psidot * state.Va * np.sin(psi))
                 + (state.Va * np.sin(psi) + we) * (psidot * state.Va * np.cos(psi))) / Vg
        f_ = np.array([[Vg * np.cos(chi)],
                       [Vg * np.sin(chi)],
                       [Vgdot],
                       [(CTRL.gravity / Vg) * np.tan(state.phi) * np.cos(chi - psi)],
                       [0.0],
                       [0.0],
                       [psidot],
                       ])
        return f_

    def h_gps(self, x, measurement, state):
        # measurement model for gps measurements
        pn = x.item(0)
        pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        h_ = np.array([
            [pn],
            [pe],
            [Vg],
            [chi],
        ])
        return h_

    def h_pseudo(self, x, measurement, state):
        # measurement model for wind triangale pseudo measurement
        pn = x.item(0)
        pe = x.item(1)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)
        h_ = np.array([
            [state.Va * np.cos(psi) + wn - Vg * np.cos(chi)],  # wind triangle x
            [state.Va * np.sin(psi) + we - Vg * np.sin(chi)],  # wind triangle y
        ])
        return h_

    def propagate_model(self, measurement, state):
        # model propagation
        for i in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, measurement, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # convert to discrete time models
            A_d = np.eye(7) + self.Ts * A + (self.Ts ** 2) * A @ A / 2.0
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Ts**2 * self.Q

    def measurement_update(self, measurement, state):
        # always update based on wind triangle pseudu measurement
        h = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0, 0]]).T
        S_inv = np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
        if (y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
            L = self.P @ C.T @ S_inv
            tmp = np.eye(7) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_pseudo @ L.T
            self.xhat = self.xhat + L @ (y - h)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            S_inv = np.linalg.inv(self.R_gps + C @ self.P @ C.T)
            if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                L = self.P @ C.T @ S_inv
                self.xhat = self.xhat + L @ (y - h)
                tmp = np.eye(7) - L @ C
                self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course

def jacobian(fun, x, measurement, state):
    # compute jacobian of fun with respect to x
    f = fun(x, measurement, state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.01  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement, state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J
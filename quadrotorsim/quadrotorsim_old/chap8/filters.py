"""
filters
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/19/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')

class alphaFilter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.alpha * self.y + (1-self.alpha) * u
        return self.y

class complementaryFilter:
    # simple complementary filter
    def __init__(self, kp=1.0, ki=0.0, Ts=0.01):
        self.kp = kp
        self.ki = ki
        self.Ts = Ts
        self.angle_hat = 0
        self.b_hat = 0

    def update(self, y_high_freq, y_low_freq):
        self.angle_hat = self.angle_hat + self.Ts * \
                         ( (y_high_freq - self.b_hat)
                             + self.kp * (y_low_freq - self.angle_hat) )
        self.b_hat = self.b_hat + self.Ts * \
                     (-self.ki * (y_low_freq - self.angle_hat))
        return self.angle_hat, self.b_hat


class kalmanFilterDiscrete:
    # Kalman filter assuming linear dynamics
    # x[k+1] = A * x[k] + B * u[k]
    # y[k] = C * x[k] + D * u[k]
    def __init__(self, A, B, C, Q, R, xhat0, P0):
        self.n = A.shape[0]
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.Q = Q
        self.R = R
        self.xhat = xhat0  # initial condition
        self.P = P0  # initial covariance

    def update(self, y, u):
        # time update
        self.P = self.A.T @ self.P @ self.A + self.Q
        self.xhat = self.A @ self.xhat + self.B @ u
        # Kalman gain
        K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)
        # measurement update
        S = (np.eye(self.n) - K @ self.C)
        self.P = S @ self.P @ S.T + K @ self.R @ K.T  # Joseph stabilized version
        self.xhat = self.xhat + K @ (y - self.C @ self.xhat)
        return self.xhat


class kalmanFilterContinuousDiscrete:
    # Continuous-discrete Kalman filter assuming linear dynamics
    # xdot = A * x + B * u
    # y[k] = C * x[k]
    def __init__(self, A, B, C, Q, R, xhat0, P0, Ts):
        self.n = A.shape[0]
        self.A = A
        self.B = B
        self.C = C
        self.Q = Q
        self.R = R
        self.xhat = xhat0  # initial condition
        self.P = P0  # initial covariance
        self.Ts = Ts
        self.N = 10  # number of integration steps between measurements

    def update(self, y, u):
        # time update
        N = 10
        for i in range(0, N):
            self.P = self.P + (self.Ts / self.N) * (self.A @ self.P + self.P @ self.A.T + self.Q)
            self.xhat = self.xhat + (self.Ts / self.N) * (self.A @ self.xhat + self.B @ u)
        # Kalman gain
        L = self.P @ self.C.T @ np.linalg.inv(self.R + self.C @ self.P @ self.C.T)
        # measurement update
        S = (np.eye(self.n) - L @ self.C)
        self.P = S @ self.P @ S.T + L @ self.R @ L.T  # Joseph stabilized version
        self.xhat = self.xhat + L @ (y - self.C @ self.xhat)
        return self.xhat

class extendedKalmanFilterContinuousDiscrete:
    # Continuous-discrete extended Kalman filter assuming dynamics
    # xdot = f(x, u)
    # y[k] = h(x[k])
    def __init__(self, f, h, Q, R, xhat0, P0, Ts):
        self.n = P0.shape[0]
        self.f = f
        self.h = h
        self.Q = Q
        self.R = R
        self.xhat = xhat0  # initial condition
        self.P = P0  # initial covariance
        self.Ts = Ts
        self.N = 10  # number of integration steps between measurements

    def update(self, y, u):
        # time update
        N = 10
        for i in range(0, N):
            A = self.df_dx(self.f, self.xhat, u)
            self.P = self.P + (self.Ts / self.N) * (A @ self.P + self.P @ A.T + self.Q)
            self.xhat = self.xhat + (self.Ts / self.N) * self.f(self.xhat, u)
        # Kalman gain
        C = self.jacobian(self.h, self.xhat)
        L = self.P @ C.T @ np.linalg.inv(self.R + C @ self.P @ C.T)
        # measurement update
        S = (np.eye(self.n) - L @ C)
        self.P = S @ self.P @ S.T + L @ self.R @ L.T  # Joseph stabilized version
        self.xhat = self.xhat + L @ (y - self.h(self.xhat))
        return self.xhat

    def df_dx(self, f, x, u):
        # Jacobian of f(x,u) with respect to x
        eps = 0.01  # deviation
        J = np.zeros((self.n, self.n))  # Jacobian of f wrt x
        f = fun(x, u)
        for i in range(0, self.n):
            x_eps = np.copy(x)
            x_eps[i][0] += eps
            f_eps = fun(x_eps, u)
            df = (f_eps - f) / eps
            J[:, i] = df[:, 0]
        return J

    def dh_dx(self, h, x):
        # Jacobian of h(x) with respect to x
        eps = 0.01  # deviation
        J = np.zeros((self.n, self.n))  # Jacobian of f wrt x
        f = fun(x, u)
        for i in range(0, self.n):
            x_eps = np.copy(x)
            x_eps[i][0] += eps
            f_eps = fun(x_eps, u)
            df = (f_eps - f) / eps
            J[:, i] = df[:, 0]
        return J
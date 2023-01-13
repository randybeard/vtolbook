"""
trajectory
    - Update history:
        3-JUL-2019 - J.B. Willis
"""

import sys
sys.path.append('..')
import numpy as np

class Trajectory:
    """ Trajectory with non-sequential access to any point along the trajectory.
        All should be parameterized by s, where s is between 0 and s_max.
    """
    def __init__(self):
        self.s_max = 100
        self.num_steps = 1000.
        self.local_search_cnt = 10 # number of steps to use when doing local search

    def getP(self, s):
        return np.array([[1, 0, 0]]).T

    def getPdot(self, s):
        return np.array([[1, 0, 0]]).T

    def getPddot(self, s):
        return np.array([[1, 0, 0]]).T
    
    def getPList(self, func, s_start, s_stop, s_step):
        pts = np.empty((3, 0))
        for s in np.arange(s_start, s_stop, s_step):
            pts = np.append(pts, func(s), axis = 1)

        return pts

    def plot3D(self, func, show=True):
        from matplotlib import pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        fig.suptitle(func.__name__[3:])
        ax = fig.add_subplot(111, projection='3d')

        pts = self.getPList(func, 0.0, self.s_max, self.s_max/1000)
        ax.plot(pts[0, :], pts[1, :], pts[2, :])
        if show:
            plt.show()

    def plot2D(self, func, show=True):
        from matplotlib import pyplot as plt
        fig = plt.figure()
        fig.suptitle(func.__name__[3:])
        s = np.arange(0.0, self.s_max, self.s_max/1000)
        pts = self.getPList(func, 0.0, self.s_max, self.s_max/1000)

        ax1 = fig.add_subplot(3, 1, 1)
        ax1.plot(s, pts[0, :])
        ax2 = fig.add_subplot(3, 1, 2)
        ax2.plot(s, pts[1, :])
        ax3 = fig.add_subplot(3, 1, 3)
        ax3.plot(s, pts[2, :])

        if show:
            plt.show()

    def findNearestP(self, pos):
        """
        Numerically search for nearest position to given position
        :return: the value of s of the nearest position found
        """
        step = self.s_max/self.num_steps # size of step along trajectory
        
        nearest, _ = self._findNearestPRange(pos, 0.0, self.s_max, step)
        return nearest

    def _findNearestPRange(self, pos, s_start, s_stop, s_step):
        nearest = 0
        nearest_dist = np.inf
        for s in np.arange(s_start, s_stop, s_step):
            # wrap s
            while s > self.s_max:
                s -= self.s_max
            while s < -self.s_max:
                s += self.s_max

            check_p = self.getP(s)
            dist = np.linalg.norm(pos - check_p)
            if dist < nearest_dist:
                nearest = s
                nearest_dist = dist

        return nearest, nearest_dist

    def findNearestPSeeded(self, pos, seed_s, step = None):
        """
        find the nearest point using a given starting point
        """
        if step is None:
            step = self.s_max/self.num_steps # size of step along trajectory

        max_offset = self.local_search_cnt*step
        nearest_fwd, dist_fwd = \
                self._findNearestPRange(pos, seed_s, seed_s + max_offset, step)
        nearest_bck, dist_bck = \
                self._findNearestPRange(pos, seed_s - max_offset, seed_s - step, step)

        if dist_fwd <= dist_bck:
            if np.abs(dist_fwd - dist_bck) <= .0001:
                return nearest_fwd
            else:
                ret = self.findNearestPSeeded(pos, nearest_fwd, step/10)
                return ret
        else:
            if np.abs(dist_bck - dist_fwd) <= .0001:
                return nearest_bck
            else:
                return self.findNearestPSeeded(pos, nearest_bck, step/10)
        

class XYZSinusoid(Trajectory):
    def __init__(self, X_max, Y_max, Z_max, X_period = 100., Y_period = 100., Z_period = 100., X_phase = 0, Y_phase = 0, Z_phase = 0):
        Trajectory.__init__(self)
        self.X_max = X_max
        self.Y_max = Y_max
        self.Z_max = Z_max

        if X_period != 0.0:
            self.X_freq = 1.0/X_period
        else:
            self.X_freq = 0.0

        if Y_period != 0.0:
            self.Y_freq = 1.0/Y_period
        else:
            self.Y_freq = 0.0

        if Z_period != 0.0:
            self.Z_freq = 1.0/Z_period
        else:
            self.Z_freq = 0.0

        self.X_phase = X_phase
        self.Y_phase = Y_phase
        self.Z_phase = Z_phase

        self.s_max = np.amax([X_period, Y_period, Z_period])

    def getP(self, s):
        X_max = self.X_max
        Y_max = self.Y_max
        Z_max = self.Z_max
        X_freq = self.X_freq
        Y_freq = self.Y_freq
        Z_freq = self.Z_freq
        X_phase = self.X_phase
        Y_phase = self.Y_phase
        Z_phase = self.Z_phase
        pi = np.pi
        return np.array([   [X_max*np.sin(X_freq*2*pi*s + X_phase)],
                            [Y_max*np.sin(Y_freq*2*pi*s + Y_phase)],
                            [Z_max*np.sin(Z_freq*2*pi*s + Z_phase)] ])

    def getPdot(self, s):
        X_max = self.X_max
        Y_max = self.Y_max
        Z_max = self.Z_max
        X_freq = self.X_freq
        Y_freq = self.Y_freq
        Z_freq = self.Z_freq
        X_phase = self.X_phase
        Y_phase = self.Y_phase
        Z_phase = self.Z_phase
        pi = np.pi
        return np.array([   [2*pi*X_freq*X_max*np.cos(X_freq*2*pi*s + X_phase)],
                            [2*pi*Y_freq*Y_max*np.cos(Y_freq*2*pi*s + Y_phase)],
                            [2*pi*Z_freq*Z_max*np.cos(Z_freq*2*pi*s + Z_phase)] ])

    def getPddot(self, s):
        X_max = self.X_max
        Y_max = self.Y_max
        Z_max = self.Z_max
        X_freq = self.X_freq
        Y_freq = self.Y_freq
        Z_freq = self.Z_freq
        X_phase = self.X_phase
        Y_phase = self.Y_phase
        Z_phase = self.Z_phase
        pi = np.pi
        return np.array([   [-((2*pi*X_freq)**2) * X_max*np.sin(X_freq*2*pi*s + X_phase)],
                            [-((2*pi*Y_freq)**2) * Y_max*np.sin(Y_freq*2*pi*s + Y_phase)],
                            [-((2*pi*Z_freq)**2) * Z_max*np.sin(Z_freq*2*pi*s + Z_phase)] ])

class HANDTraj(Trajectory):
    def __init__(self):
        Trajectory.__init__(self)
        import trajectory_planning.hand_traj_params as HT
        self.pdd_x = HT.ax
        self.pd_x = HT.vx
        self.p_x = HT.px

        self.pdd_y = HT.ay
        self.pd_y = HT.vy
        self.p_y = HT.py

        self.pdd_z = -HT.az
        self.pd_z = -HT.vz
        self.p_z = -HT.pz
        
        self.s_max = HT.t_end - HT.delta_t
        self.s_step = HT.delta_t
        self.num_steps = HT.num_points

    def getP(self, s):
        s = int(s/self.s_step)
        return np.array([   [self.p_x[s]],
                            [self.p_y[s]],
                            [self.p_z[s]] ])

    def getPdot(self, s):
        s = int(s/self.s_step)
        return np.array([   [self.pd_x[s]],
                            [self.pd_y[s]],
                            [self.pd_z[s]] ])

    def getPddot(self, s):
        s = int(s/self.s_step)
        return np.array([   [self.pdd_x[s]],
                            [self.pdd_y[s]],
                            [self.pdd_z[s]] ])
                            


if __name__ == "__main__":
    # traj = XYZSinusoid(20, 40, 10, 50, 100, 100, 0, np.pi/2, np.pi)
    traj = HANDTraj()
    traj.plot2D(traj.getP, show=False)
    traj.plot2D(traj.getPdot, show=False)
    traj.plot2D(traj.getPddot, show=False)
    traj.plot3D(traj.getP, show=False)
    from matplotlib import pyplot as plt
    plt.show()

    pos = np.array([[0, 0, 0]]).T
    near_s = traj.findNearestP(pos)
    near_pos = traj.getP(near_s)
    print("Nearest position to {} is {}".format(pos.T, near_pos.T))


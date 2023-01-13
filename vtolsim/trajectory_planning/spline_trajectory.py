import sys
sys.path.append('..')
from trajectory_planning.trajectory import Trajectory
import numpy as np
import trajectory_planning.splines as spl


class SplineTrajectory(Trajectory):
    def __init__(self, knots, v_arr):
        """
        param knots: a nx3 array of the knot locations
        param v_arr: a nx3 array of the desired velocity between knots
        """

        Trajectory.__init__(self)

        PD_list = spl.computeSplineDArray(knots)

        self.knots = knots
        self.PD_list = PD_list
        self.vels = v_arr
        
        # s_max is the maximum value of the normal parameterization
        self.sbar_max = self.knots.shape[0]-1.001
        self.s_max = self.sbar_max

    def getP(self, sbar):
        sbar = self._sat_s(sbar)
        sseg, sknot = spl.getSegKnot_s(sbar)
        
        a, b, c, d = self._getABCD(sknot)
        P = a + b*sseg + c*sseg**2 + d*sseg**3

        return np.array([   [P.item(0)],
                            [P.item(1)],
                            [P.item(2)] ])

    def getPdot(self, sbar):
        sbar = self._sat_s(sbar)
        sseg, sknot = spl.getSegKnot_s(sbar)
        
        a, b, c, d = self._getABCD(sknot)

        vi   = self.vels[sknot, :]
        vip1 = self.vels[sknot+1, :]

        # linear interpolate between vi and vi+1
        V = (1-sseg)*vi + sseg*vip1 

        Pd = (b + 2*c*sseg + 3*d*sseg**2)
        Pd = V*Pd/np.linalg.norm(Pd)

        return np.array([   [Pd.item(0)],
                            [Pd.item(1)],
                            [Pd.item(2)] ])

    def getPddot(self, sbar):
        print("WARNING: Spline get Pddot is not correct, though unused")
        sbar = self._sat_s(sbar)
        sseg, sknot = spl.getSegKnot_s(sbar)
        
        a, b, c, d = self._getABCD(sknot)
        Pdd = (2*c + 6*d*sseg)

        return np.array([   [Pdd.item(0)],
                            [Pdd.item(1)],
                            [Pdd.item(2)] ])


    def _sat_s(self, sbar):
        if sbar < 0.0:
           return 0.0
        elif sbar > self.sbar_max:
            return self.sbar_max
        else:
            return sbar



    def _getABCD(self, sknot): 

        yi   = self.knots[sknot, :]
        yip1 = self.knots[sknot+1, :]
        Di   = self.PD_list[sknot, :]
        Dip1 = self.PD_list[sknot+1, :]

        return spl.getABCD(yi, yip1, Di, Dip1)

if __name__ == "__main__":

    import parameters.spline_parameters as SPP

    traj = SplineTrajectory(SPP.pts, SPP.vels)

    traj.plot2D(traj.getP, show=False)
    traj.plot2D(traj.getPdot, show=False)
    traj.plot2D(traj.getPddot, show=False)
    traj.plot3D(traj.getP)

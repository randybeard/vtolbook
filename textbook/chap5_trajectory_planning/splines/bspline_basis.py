"""
messing with b-splines
        2/18/21 - RWB
        10/19/22 - RWB
"""
import numpy as np
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt

def splineBasis(degree, knots, controlPoints, time):
        spl = BSpline(knots, controlPoints, degree)
        basisPoints = spl(time)
        return( basisPoints )

def uniformClampedKnots(degree, M):
        knots = [0] * degree + list(range(0, M+1)) + [M] * degree
        return knots

def uniformKnots(degree, M):
        knots = list(range(-degree, M+degree+1))
        return knots

def plotSplineBasis(degree, M, clamped=True):
        fig = plt.figure(degree)
        t = np.linspace(0, M, 100)
        if clamped:
                knots = uniformClampedKnots(degree, M)
        else:
                knots = uniformKnots(degree, M)
        fig.suptitle(f"degree={degree}, knots = {str(knots)}")
        ax = fig.subplots(M+degree)
        for i in range(0, M+degree):
                ctrl = [0] * (M+degree)
                ctrl[i] = 1
                pts = splineBasis(degree, knots, ctrl, t)
                ax[i].plot(t, pts)
                ax[i].set(ylabel=f"m={i}")

plotSplineBasis(degree=0, M=2)
#plotSplineBasis(degree=1, M=2, clamped=True)
plotSplineBasis(degree=1, M=2, clamped=False)
#plotSplineBasis(degree=2, M=3, clamped=True)
plotSplineBasis(degree=2, M=3, clamped=False)
plt.show()



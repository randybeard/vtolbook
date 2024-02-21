"""
messing with b-splines
        2/18/21 - RWB
        10/19/22 - RWB
"""
import numpy as np
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def splineBasis(order, knots, controlPoints, time):
        spl = BSpline(knots, controlPoints, order)
        basisPoints = spl(time)
        return( basisPoints )

def uniformClampedKnots(order, M):
        knots = [0] * order + list(range(0, M+1)) + [M] * order
        return knots

def uniformKnots(order, M):
        knots = list(range(-order, M+order+1))
        return knots

def plotSplineBasis(order, M, clamped=True):
        fig = plt.figure(order)
        t = np.linspace(0, M, 100)
        if clamped:
                knots = uniformClampedKnots(order, M)
        else:
                knots = uniformKnots(order, M)
        fig.suptitle(f"Order={order}, knots = {str(knots)}")
        ax = fig.subplots(M+order)
        for i in range(0, M+order):
                ctrl = [0] * (M+order)
                ctrl[i] = 1
                pts = splineBasis(order, knots, ctrl, t)
                ax[i].plot(t, pts)
                ax[i].set(ylabel=f"m={i}")

plotSplineBasis(order=0, M=2)
#plotSplineBasis(order=1, M=2, clamped=True)
plotSplineBasis(order=1, M=2, clamped=False)
#plotSplineBasis(order=2, M=3, clamped=True)
plotSplineBasis(order=2, M=3, clamped=False)
plt.show()



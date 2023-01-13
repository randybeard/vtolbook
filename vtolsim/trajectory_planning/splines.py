"""
    splines
    Date Started: 28-Aug-2019
    Jacob Willis
"""

import sys
sys.path.append('..')
import numpy as np

def computeSplineMatrix(n):
  mat = np.diag(np.ones(n-1), k=1) + 4*np.eye(n) + np.diag(np.ones(n-1), k=-1)
  mat[0,0] = 2
  mat[n-1,n-1] = 2
  mat
  return mat

def computeSplinePtsArray(pts):
  # right hand side of matrix spline equation
  n = pts.shape[0]
  l = pts.shape[1]
  arr = np.zeros((n,l))
  arr[0,:] = 3*(pts[1,:] - pts[0,:])
  for i in range(1, n-1):
    ai = 3*(pts[i+1,:] - pts[i-1,:])
    arr[i,:] = ai
  arr[n-1,:] = 3*(pts[n-1,:] - pts[n-2,:])
  return arr
    
def computeSplineDArray(pts):
  n = pts.shape[0]
  m = n-1 # number of segments
  splineMat = computeSplineMatrix(n)
  ptsArray = computeSplinePtsArray(pts)
  
  D = np.linalg.solve(splineMat, ptsArray)
  
  return D

def computePdot(u, a, b, c, d):
  return b + 2*c*u + 3*d*u**2 

def computeSegPdotMax(yi, yip1, Di, Dip1):

  a, b, c, d = getABCD(yi, yip1, Di, Dip1)

  # compute possible maximums s = (0, 1, zero of pdd)
  pd_0 = computePdot(0, a, b, c, d)
  pd_1 = computePdot(1, a, b, c, d)
  if not d == 0.0: # if d is zero, s has a constant second derivative
    u = -c/(3*d)
    pd_2 = computePdot(u, a, b, c, d)
  else:
    pd_2 = 0.0

  return float(np.max(np.abs([pd_0, pd_1, pd_2])))

def computeSplinePdotMax(pts, D):
  maxFound = 0.0
  n = pts.shape[0]
  for i in range(0, n-2):
    l = pts.shape[1]

    # find the max for the three columns
    # this may actually be higher than is true
    max_i = 0.0
    for j in range(0, l):
        max_ij = computeSegPdotMax(pts[i, j], 
                                  pts[i+1, j], 
                                  D[i, j],
                                  D[i+1, j])
        max_i += max_ij**2

    max_i = np.sqrt(max_i)
    
    if max_i > maxFound:
      maxFound = max_i
    
  return maxFound

def computeSplineParameters(pts, max_v):
    """
    helper to get everything done at once
    """
    D = computeSplineDArray(pts)
    pdotmax = computeSplinePdotMax(pts, D)
    alpha = max_v/pdotmax
    return D, alpha

def getSegKnot_s(sbar):
    sseg = sbar % 1.0 # the position along this segment (between 0 and 1)
    sknot = int(sbar) # the index of the knot starting the current segment

    return sseg, sknot

def getABCD(yi, yip1, Di, Dip1):
  a = yi
  b = Di
  c = 3*(yip1 - yi) - 2*Di - Dip1
  d = 2*(yi - yip1) + Di + Dip1
  
  return a, b, c, d

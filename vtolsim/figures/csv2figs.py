#! /usr/bin/env python3
"""
Generate desired figures for the ACC 2020 paper using data from a CSV file
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

parser = argparse.ArgumentParser(description='Generate desired figures for the ACC 2020 paper using data from a CSV file')

parser.add_argument('file', help="csv file containing data to be plotted")

args = parser.parse_args()

print("file = ", args.file)
data = np.loadtxt(args.file, delimiter=',', unpack=True)
t    = data[:, 0]
pn   = data[:, 1]
pe   = data[:, 2]
h    = data[:, 3]
u    = data[:, 4]
v    = data[:, 5]
w    = data[:, 6]
pn_d = data[:, 7]
pe_d = data[:, 8]
h_d  = data[:, 9]
u_d  = data[:, 10]
v_d  = data[:, 11]
w_d  = data[:, 12]


################################################################################
## Plot 1: position in 3 subplots over time
fig = plt.figure(1)
ax_n = fig.add_subplot(311)
ax_e = fig.add_subplot(312)
ax_h = fig.add_subplot(313)

fig.suptitle("Position")
ax_n.plot(t, pn, label = 'actual')
ax_n.plot(t, pn_d, label = 'desired')
ax_n.set_xlabel("$t$")
ax_n.set_ylabel("north (m)")

ax_e.plot(t, pe, label = 'actual')
ax_e.plot(t, pe_d, label = 'desired')
ax_e.set_xlabel("$t$")
ax_e.set_ylabel("east (m)")

ax_h.plot(t, h, label = 'actual')
ax_h.plot(t, h_d, label = 'desired')
ax_h.set_xlabel("$t$")
ax_h.set_ylabel("height (m)")

################################################################################
## Plot 2: 3d plot of desired and actual position
fig = plt.figure(2)
fig.suptitle("3D Position")
ax = fig.add_subplot(111, projection='3d')

ax.plot(pn, pe, h, label='actual')
ax.plot(pn_d, pe_d, h_d, label='desired')

################################################################################
## Plot 3: Position error plot

p_err = np.sqrt((pn_d - pn)**2 + (pe_d - pe)**2 + (h_d - h)**2)

fig = plt.figure(3)
ax = fig.add_subplot(111)

fig.suptitle("Position Error")
ax.plot(t, p_err)
ax.set_xlabel("$t$")
ax.set_ylabel("$error (m)")


################################################################################
## Plot 4: velocity error plot


plt.show()

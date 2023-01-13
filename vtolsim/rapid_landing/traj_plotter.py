#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np

# from minimum_landing_time_calculator import MinimumTimeLandingCalculator
from minimum_landing_time_2d import MinimumTimeLandingCalculator2D
from minimum_landing_time_2d import LandingRoutine

# mtlc = MinimumTimeLandingCalculator2D()
mtlc = LandingRoutine()
# mtlc.reposition_start_at_origin()
# times = np.linspace(mtlc.landing_entrance.t_start_downward_turn,
#     mtlc.landing_entrance.t_start_constant_descent, 1000)
times = np.linspace(0, mtlc.t_f, 1000)
positions = []
velocities = []
accelerations = []

for t in times:
    positions.append(mtlc.position_function(t))
    velocities.append(mtlc.velocity_function(t))
    # accelerations.append(mtlc.acceleration_function(t))

positions = np.array(positions)
velocities = np.array(velocities)
accelerations = np.array(accelerations)

plt.plot(positions[:,0], positions[:,1])
plt.show()
exit(0)

plt.subplot(321)
plt.plot(times, positions[:,0])
plt.subplot(322)
plt.plot(times, positions[:,1])

# plt.subplot(233)
# plt.plot(times, np.degrees(np.arctan2(positions[:,1],positions[:,0])))
# plt.plot(positions[:,0], -positions[:,1])
# plt.show()

plt.subplot(323)
plt.plot(times, velocities[:,0])
plt.subplot(324)
plt.plot(times, velocities[:,1])

plt.show()
exit(0)
# plt.subplot(233)
# plt.plot(times, (velocities[:,1]/velocities[:,0]))
# plt.plot(positions[:,0], -positions[:,1])

plt.subplot(325)
plt.plot(times, accelerations[:,0])
plt.subplot(326)
plt.plot(times, accelerations[:,1])
plt.show()


# times = np.linspace(0, 12, 12)
# waypoints = mtlc.get_xyz_points(np.pi/6, times)
# print(times)
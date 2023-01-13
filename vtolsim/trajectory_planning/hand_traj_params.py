import numpy as np

t_end = 40.0
num_points = t_end/.02
delta_t = t_end/num_points
t = np.linspace(0.0, t_end, num_points)

ax = np.zeros(t.shape)
vx = np.zeros(t.shape)
px = np.zeros(t.shape)

ay = np.zeros(t.shape)
vy = np.zeros(t.shape)
py = np.zeros(t.shape)

az = np.zeros(t.shape)
vz = np.zeros(t.shape)
pz = np.zeros(t.shape)

ax[0:int(num_points*0.25)] = 4
ax[int(num_points*0.25):int(num_points*0.75)] = 0
ax[int(num_points*0.5):int(num_points*0.8)] = -3
ax[int(num_points*0.8):int(num_points*0.9)] = -4
ax[int(num_points*0.9):] = 5
# ax[int(num_points*0.95):] = -3

ay[0:int(num_points*0.8)] = 0
ay[int(num_points*0.8):int(num_points*0.85)] = 8
ay[int(num_points*0.85):int(num_points*0.95)] = -8
ay[int(num_points*0.95):] = 8

az[0:int(num_points*0.2)] = 0.375
az[int(num_points*0.2):int(num_points*0.4)] = -0.375
az[int(num_points*0.4):int(num_points*0.8)] = 0
az[int(num_points*0.7):int(num_points*0.85)] = -2/3
az[int(num_points*0.85):] = 2/3


for i in range(t.shape[0]):
  if i == 0:
    vx[i] = 0.0
    px[i] = 0.0
    vy[i] = 0.0
    py[i] = 0.0
    vz[i] = 0.0
    pz[i] = 0.0
  else:
    vx[i] = vx[i-1] + ax[i] * delta_t
    px[i] = px[i-1] + vx[i-1] * delta_t + 0.5 * ax[i] * delta_t**2
    vy[i] = vy[i-1] + ay[i] * delta_t
    py[i] = py[i-1] + vy[i-1] * delta_t + 0.5 * ay[i] * delta_t**2
    vz[i] = vz[i-1] + az[i]*(t[i] - t[i-1])
    pz[i] = pz[i-1] + vz[i-1] * delta_t + 0.5 * az[i] * delta_t**2


ax = ax / 4.
vx = vx / 4.
px = px / 4.

ay = ay / 2.
vy = vy / 2.
py = py / 2.

az = az / 4.
vz = vz / 4.
pz = pz / 4.

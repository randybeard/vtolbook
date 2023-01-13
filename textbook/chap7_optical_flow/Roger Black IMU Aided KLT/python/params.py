# time between measurements/frames
dt = 1/30

# how many frames to skip between warping
skip = 2

# focal length todo: get this from the camera using get_intrinsics()?
# a value of 500 for both the focal length and distance has produced good results
f = 500

# z distance
dist = 500

# the alpha values used in the low pass filter.
# separate parameters for acceleration and gyro in case it is determined they should be different
accel_alpha = 0.5
gyro_alpha = accel_alpha

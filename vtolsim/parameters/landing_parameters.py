import numpy as np

V_trans = 13
# A positive V_f represents a positive downward velocity
V_f = 0
gamma = -np.radians(10)
p_f = np.array([[0], [0]])
gain = 1.0

# Landing Approach
horizontal_flight_dist = 100
height = 20
circle_r = 500
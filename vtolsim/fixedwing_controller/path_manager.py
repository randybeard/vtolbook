from csv import QUOTE_NONNUMERIC
import numpy as np
import sys
sys.path.append('..')
from message_types.msg_path import MsgPath

FILLET_LINE_ST = 1
FILLET_ORBIT_ST = 2

DUBINS_CIRCLE_1_PRE_ST = 1
DUBINS_CIRCLE_1_POST_ST = 2
DUBINS_STRAIGHT_ST = 3
DUBINS_CIRCLE_2_PRE_ST = 4
DUBINS_CIRCLE_2_POST_ST = 5

class PathManager:
    def __init__(self):
        # message sent to path follower
        self.path = MsgPath()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        self.manager_requests_waypoints = True

    def update(self, waypoints, radius, state):
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def initialize_pointers(self):
        if self.num_waypoints >= 3:
            self.ptr_previous = 0
            self.ptr_current = 1
            self.ptr_next = 2
        else:
            print('Error Path Manager: need at least three waypoints')
            assert True

    def increment_pointers(self):
        self.ptr_previous = self.ptr_previous + 1 if self.ptr_previous < self.num_waypoints - 1 else 0
        self.ptr_current = self.ptr_current + 1 if self.ptr_current < self.num_waypoints - 1 else 0
        self.ptr_next = self.ptr_next + 1 if self.ptr_next < self.num_waypoints - 1 else 0

    def in_halfspace(self, pos):
        print(pos)
        print(self.halfspace_n)
        print(self.halfspace_r)
        print((pos-self.halfspace_r).T @ self.halfspace_n)
        assert (np.shape((pos-self.halfspace_r).T @ self.halfspace_n) == (1, 1)), \
            f'halfspace shape: {np.shape((pos-self.halfspace_r).T @ self.halfspace_n)}'
        if (((pos-self.halfspace_r).T @ self.halfspace_n).item(0) >= 0.0): #implement code here
            return True
        else:
            return False

    def get_three_waypoints(self, waypoints):
        w_prev = np.reshape(waypoints.ned[:, self.ptr_previous], (3,1))
        w_curr = np.reshape(waypoints.ned[:, self.ptr_current], (3,1))
        w_next = np.reshape(waypoints.ned[:, self.ptr_next], (3,1))
        return w_prev, w_curr, w_next

    def get_qs(self, w_prev, w_curr, w_next):
        q_prev = (w_curr - w_prev) / np.linalg.norm(w_curr - w_prev)
        q_curr = (w_next - w_curr) / np.linalg.norm(w_next - w_curr)
        return q_prev, q_curr

    def line_manager(self, waypoints, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed:
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
        # state machine for line path
        w_prev, w_curr, w_next = self.get_three_waypoints(waypoints)
        q_prev, q_curr = self.get_qs(w_prev, w_curr, w_next)
        self.halfspace_r = w_curr
        self.halfspace_n = (q_prev + q_curr) / np.linalg.norm(q_prev + q_curr)
        if self.in_halfspace(mav_pos):
            self.increment_pointers()

        # Update path
        self.path = MsgPath()
        w_prev, w_curr, w_next = self.get_three_waypoints(waypoints)
        q_prev, _ = self.get_qs(w_prev, w_curr, w_next)
        self.path.line_origin = w_prev
        self.path.line_direction = q_prev

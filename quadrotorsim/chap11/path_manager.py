import numpy as np
import sys
sys.path.append('..')
from chap11.dubins_parameters import dubinsParameters
from message_types.msg_path import msgPath

class pathManager:
    def __init__(self):
        # message sent to path follower
        self.path = msgPath()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        # flag that request new waypoints from path planner
        self.flag_need_new_waypoints = True
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        # dubins path parameters
        self.dubins_path = dubinsParameters()

    def update(self, waypoints, radius, state):
        # this flag is set for one time step to signal a redraw in the viewer
        if self.path.flag_path_changed == True:
            self.path.flag_path_changed = False
        if waypoints.num_waypoints == 0:
            self.flag_need_new_waypoints = True
        else:
            self.flag_need_new_waypoints = False
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def line_manager(self, waypoints, state):
        mav_pos = np.array([[state.pn, state.pe, -state.h]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed == True:
            waypoints.flag_manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            self.construct_line(waypoints)

        # entered into the half plane separating waypoint segments
        if self.inHalfSpace(mav_pos):
            self.increment_pointers()
            self.construct_line(waypoints)

    def fillet_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.pn, state.pe, -state.h]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed == True:
            waypoints.flag_manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            self.construct_fillet_line(waypoints, radius)
            self.manager_state = 1
        # state machine for fillet path
        if self.manager_state == 1:
            # follow straight line path from previous to current
            if self.inHalfSpace(mav_pos):
                # entered into the half plane H1
                self.construct_fillet_circle(waypoints, radius)
                self.manager_state = 2
        elif self.manager_state == 2:
            # follow orbit from previous->current to current->next
             if self.inHalfSpace(mav_pos):
                # entered into the half plane H2
                self.increment_pointers()
                self.construct_fillet_line(waypoints, radius)
                self.manager_state = 1

    def dubins_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.pn, state.pe, -state.h]]).T
        # if the waypoints have changed, update the waypoint pointer
        if waypoints.flag_waypoints_changed == True:
            waypoints.flag_manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False
            self.num_waypoints = waypoints.num_waypoints
            self.initialize_pointers()
            self.dubins_path.update(
                waypoints.ned[:, self.ptr_previous:self.ptr_previous+1],
                waypoints.course.item(self.ptr_previous),
                waypoints.ned[:, self.ptr_current:self.ptr_current+1],
                waypoints.course.item(self.ptr_current),
                radius)
            self.construct_dubins_circle_start(waypoints, radius)
            if self.inHalfSpace(mav_pos):
                self.manager_state = 1
            else:
                self.manager_state = 2
        # state machine for dubins path
        if self.manager_state == 1:
            # follow start orbit until out of H1
            if not self.inHalfSpace(mav_pos):
                 self.manager_state = 2
        elif self.manager_state == 2:
            # follow start orbit until cross into H1
             if self.inHalfSpace(mav_pos):
                 self.construct_dubins_line(waypoints)
                 self.manager_state = 3
        elif self.manager_state == 3:
            # follow start orbit until cross into H2
             if self.inHalfSpace(mav_pos):
                 self.construct_dubins_circle_end(waypoints, radius)
                 if self.inHalfSpace(mav_pos):
                     self.manager_state = 4
                 else:
                     self.manager_state = 5
        elif self.manager_state == 4:
            # follow start orbit until out of H3
            if not self.inHalfSpace(mav_pos):
                self.manager_state = 5
        elif self.manager_state == 5:
            # follow start orbit until cross into H3
             if self.inHalfSpace(mav_pos):
                self.increment_pointers()
                self.dubins_path.update(
                    waypoints.ned[:, self.ptr_previous:self.ptr_previous+1],
                    waypoints.course.item(self.ptr_previous),
                    waypoints.ned[:, self.ptr_current:self.ptr_current+1],
                    waypoints.course.item(self.ptr_current),
                    radius)
                self.construct_dubins_circle_start(waypoints, radius)
                if self.inHalfSpace(mav_pos):
                    self.manager_state = 1
                else:
                    self.manager_state = 2

    def initialize_pointers(self):
        if self.num_waypoints >= 3:
            self.ptr_previous = 0
            self.ptr_current = 1
            self.ptr_next = 2
        else:
            print('Error Path Manager: need at least three waypoints')

    def increment_pointers(self):
        self.ptr_previous = self.ptr_current
        if self.ptr_current == self.num_waypoints-2:
            self.ptr_current = self.num_waypoints-1
            self.ptr_next = 0
        elif self.ptr_current == self.num_waypoints-1:
            self.ptr_current = 0
            self.ptr_next = 1
        else:
            self.ptr_current = self.ptr_next
            self.ptr_next = self.ptr_next + 1

    def construct_line(self, waypoints):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.line_origin = previous
        q_previous = current - previous
        q_previous = q_previous / np.linalg.norm(q_previous)
        self.path.line_direction = q_previous
        q_next   = next - current
        q_next   = q_next / np.linalg.norm(q_next)
        self.halfspace_n = (q_previous + q_next) / 2
        self.halfspace_n = self.halfspace_n / np.linalg.norm(self.halfspace_n)
        self.halfspace_r = current
        self.path.flag_path_changed = True

    def construct_fillet_line(self, waypoints, radius):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.line_origin = previous
        q_previous = current - previous
        q_previous = q_previous / np.linalg.norm(q_previous)
        self.path.line_direction = q_previous
        q_next   = (next-current) / np.linalg.norm(next-current)
        beta = np.arccos(-q_previous.T @ q_next)
        self.halfspace_n = q_previous
        self.halfspace_r = current - radius / np.tan(beta/2) * q_previous
        self.path.flag_path_changed = True

    def construct_fillet_circle(self, waypoints, radius):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        self.path.type = 'orbit'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        q_previous = (current - previous) / np.linalg.norm(current - previous)
        q_next   = (next - current) / np.linalg.norm(next - current)
        beta = np.arccos(-q_previous.T @ q_next)
        self.path.orbit_center = current \
                                 - radius / np.sin(beta/2) * (q_previous - q_next) / np.linalg.norm(q_previous - q_next)
        self.path.orbit_radius = radius
        if np.sign(q_previous.item(0)*q_next.item(1) - q_previous.item(1) - q_next.item(0)) > 0:
            self.path.orbit_direction = 'CW'
        else:
            self.path.orbit_direction = 'CCW'
        self.halfspace_n = q_next
        self.halfspace_r = current + radius / np.tan(beta/2) * q_next
        self.path.flag_path_changed = True

    def construct_dubins_circle_start(self, waypoints, radius):
        self.path.type = 'orbit'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.orbit_radius = radius
        self.path.orbit_center = self.dubins_path.center_s
        if self.dubins_path.dir_s == 1:
            self.path.orbit_direction = 'CW'
        else:
            self.path.orbit_direction = 'CCW'
        self.halfspace_n = self.dubins_path.n1
        self.halfspace_r = self.dubins_path.r1
        self.path.flag_path_changed = True

    def construct_dubins_line(self, waypoints):
        self.path.type = 'line'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.line_origin = self.dubins_path.r1
        self.path.line_direction = self.dubins_path.n1
        self.halfspace_n = self.dubins_path.n1
        self.halfspace_r = self.dubins_path.r2
        self.path.flag_path_changed = True

    def construct_dubins_circle_end(self, waypoints, radius):
        self.path.type = 'orbit'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.orbit_radius = radius
        self.path.orbit_center = self.dubins_path.center_e
        if self.dubins_path.dir_e == 1:
            self.path.orbit_direction = 'CW'
        else:
            self.path.orbit_direction = 'CCW'
        self.halfspace_n = self.dubins_path.n3
        self.halfspace_r = self.dubins_path.r3
        self.path.flag_path_changed = True

    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        else:
            return False


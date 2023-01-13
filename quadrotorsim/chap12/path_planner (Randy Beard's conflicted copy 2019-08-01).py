# path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - BGM
import numpy as np
import sys
sys.path.append('..')
from message_types.msg_waypoints import msgWaypoints
from chap12.rrt_straight_line import rrtStraightLine
from chap12.rrt_dubins import rrtDubins

class pathPlanner:
    def __init__(self):
        # waypoints definition
        self.waypoints = msgWaypoints()
        self.rrt_straight_line = rrtStraightLine()
        self.rrt_dubins = rrtDubins()

    def update(self, map, state, radius):
        # this flag is set for one time step to signal a redraw in the viewer
        # planner_flag = 1  # return simple waypoint path
        #planner_flag = 2  # return dubins waypoint path
        #planner_flag = 3  # plan path through city using straight-line RRT
        planner_flag = 4  # plan path through city using dubins RRT
        if planner_flag == 1:
            Va = 25
            self.waypoints.type = 'fillet'
            self.waypoints.add(np.array([[0, 0, -100]]).T, Va, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[0, 1000, -100]]).T, Va, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 1000, -100]]).T, Va, np.inf, np.inf, 0, 0)
        elif planner_flag == 2:
            Va = 25
            self.waypoints.type = 'dubins'
            self.waypoints.add(np.array([[0, 0, -100]]).T, Va, np.radians(0), np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
            self.waypoints.add(np.array([[0, 1000, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 1000, -100]]).T, Va, np.radians(-135), np.inf, 0, 0)
        elif planner_flag == 3:
            Va = 25
            h_d = 100
            # start pose is current pose
            start_pose = np.array([[state.pn], [state.pe], [-h_d]])
            # desired end pose
            end_pose = np.array([[map.city_width], [map.city_width], [-h_d]])
            if np.linalg.norm(start_pose-end_pose) < map.city_width / 2:
                end_pose = np.array([[0], [0], [-h_d]])
            self.waypoints.type = 'fillet'
            self.waypoints = self.rrt_straight_line.update(start_pose, end_pose, Va, map, radius)
        elif planner_flag == 4:
            Va = 25
            h_d = 100
            # start pose is current pose
            start_pose = np.array([[state.pn], [state.pe], [-h_d], [state.chi]])
            # desired end pose
            end_pose = np.array([[map.city_width], [map.city_width], [-h_d], [state.chi]])
            if np.linalg.norm(start_pose[0:3]-end_pose[0:3]) < map.city_width / 2:
                end_pose = np.array([[0], [0], [-h_d], [state.chi]])
            self.waypoints.type = 'fillet'
            self.waypoints = self.rrt_dubins.update(start_pose, end_pose, Va, map, radius)
        else:
            print("Error in Path Planner: Undefined planner type.")

        return self.waypoints

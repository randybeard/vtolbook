# rrt dubins path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/16/2019 - RWB
import numpy as np
from message_types.msg_waypoints import msgWaypoints
from viewers.drawing import drawWaypoints, drawMap
from chap11.dubins_parameters import dubinsParameters
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class rrtDubins():
    def __init__(self):
        self.segment_length = 500  # standard length of path segments
        self.tree = msgWaypoints()
        self.tree.type = 'dubins'
        self.plot_window = []
        self.plot_app = []
        self.dubins_path = dubinsParameters()

    def update(self, start_pose, end_pose, Va, map, radius):
        # add the start pose to the tree
        self.tree.add(start_pose[0:3], Va, start_pose.item(3), 0, 0, 0)
        # check to see if start_pose connects directly to end_pose
        if self.distance(start_pose, end_pose) < self.segment_length\
            and self.distance(start_pose, end_pose) >= 2 * radius\
                and self.collision(start_pose, end_pose, map, radius) is False:
            self.tree.add(end_pose[0:3], Va, end_pose.item(3), self.distance(start_pose, end_pose), 1, 1)
        else:
            num_paths = 0
            while num_paths < 3:
                flag = self.extend_tree(end_pose, Va, map, radius)
                num_paths = num_paths + flag
        # find path with minimum cost to end_node
        waypoints_not_smooth = self.find_minimum_path(end_pose)
        waypoints = self.smooth_path(waypoints_not_smooth, map, radius)
        self.plot_map(map, waypoints_not_smooth, waypoints, radius)
        return waypoints

    def distance(self, start_pose, end_pose):
        # compute distance between start and end pose
        d = np.linalg.norm(start_pose[0:3] - end_pose[0:3])
        return d

    def collision(self, start_pose, end_pose, map, radius):
        # check to see of path from start_pose to end_pose colliding with map
        collision_flag = False
        Del = 0.05
        self.dubins_path.update(start_pose[0:3], start_pose.item(3), end_pose[0:3], end_pose.item(3), radius)
        points = self.points_along_path(Del)
        for i in range(points.shape[1]):
            if self.height_above_ground(map, column(points, i)) <= 0:
                collision_flag = True
        return collision_flag

    def points_along_path(self, Del):
        # returns a list of points along the dubins path
        initialize_points = True
        # points along start circle
        th1 = np.arctan2(self.dubins_path.p_s.item(1) - self.dubins_path.center_s.item(1),
                         self.dubins_path.p_s.item(0) - self.dubins_path.center_s.item(0))
        th1 = mod(th1)
        th2 = np.arctan2(self.dubins_path.r1.item(1) - self.dubins_path.center_s.item(1),
                         self.dubins_path.r1.item(0) - self.dubins_path.center_s.item(0))
        th2 = mod(th2)
        th = th1
        theta_list = [th]
        if self.dubins_path.dir_s > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2:
                    th -= Del
                    theta_list.append(th)

        if initialize_points:
            points = np.array([[self.dubins_path.center_s.item(0) + self.dubins_path.radius * np.cos(theta_list[0]),
                                self.dubins_path.center_s.item(1) + self.dubins_path.radius * np.sin(theta_list[0]),
                                self.dubins_path.center_s.item(2)]])
            initialize_points = False
        for angle in theta_list:
            new_point = np.array([[self.dubins_path.center_s.item(0) + self.dubins_path.radius * np.cos(angle),
                                   self.dubins_path.center_s.item(1) + self.dubins_path.radius * np.sin(angle),
                                   self.dubins_path.center_s.item(2)]])
            points = np.concatenate((points, new_point), axis=0)

        # points along straight line
        sig = 0
        while sig <= 1:
            new_point = np.array([[(1 - sig) * self.dubins_path.r1.item(0) + sig * self.dubins_path.r2.item(0),
                                   (1 - sig) * self.dubins_path.r1.item(1) + sig * self.dubins_path.r2.item(1),
                                   (1 - sig) * self.dubins_path.r1.item(2) + sig * self.dubins_path.r2.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
            sig += Del

        # points along end circle
        th2 = np.arctan2(self.dubins_path.p_e.item(1) - self.dubins_path.center_e.item(1),
                         self.dubins_path.p_e.item(0) - self.dubins_path.center_e.item(0))
        th2 = mod(th2)
        th1 = np.arctan2(self.dubins_path.r2.item(1) - self.dubins_path.center_e.item(1),
                         self.dubins_path.r2.item(0) - self.dubins_path.center_e.item(0))
        th1 = mod(th1)
        th = th1
        theta_list = [th]
        if self.dubins_path.dir_e > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2:
                    th -= Del
                    theta_list.append(th)
        for angle in theta_list:
            new_point = np.array([[self.dubins_path.center_e.item(0) + self.dubins_path.radius * np.cos(angle),
                                   self.dubins_path.center_e.item(1) + self.dubins_path.radius * np.sin(angle),
                                   self.dubins_path.center_e.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
        return points

    def height_above_ground(self, map, point):
        # find the altitude of point above ground level
        point_height = -point.item(2)
        tmp = np.abs(point.item(0)-map.building_north)
        d_n = np.min(tmp)
        idx_n = np.argmin(tmp)
        tmp = np.abs(point.item(1)-map.building_east)
        d_e = np.min(tmp)
        idx_e = np.argmin(tmp)
        if (d_n<map.building_width) and (d_e<map.building_width):
            map_height = map.building_height[idx_n, idx_e]
        else:
            map_height = 0
        h_agl = point_height - map_height
        return h_agl

    def extend_tree(self, end_pose, Va, map, radius):
        # extend tree by randomly selecting pose and extending tree toward that pose
        flag1 = False
        while flag1 is False:
            random_pose = self.random_pose(map, end_pose.item(2))
            # find leaf on tree that is closest to random_pose
            tmp = self.tree.ned-np.tile(random_pose[0:3], (1, self.tree.num_waypoints))
            tmp1 = np.diag(tmp.T @ tmp)
            idx = np.argmin(tmp1)
            dist = tmp1.item(idx)
            L = np.max([np.min([np.sqrt(dist), self.segment_length]), 3*radius])
            cost = self.tree.cost.item(idx) + L
            tmp = random_pose[0:3]-column(self.tree.ned, idx)
            new_ned = column(self.tree.ned, idx) + L * (tmp / np.linalg.norm(tmp))
            new_chi = np.arctan2(new_ned.item(1) - self.tree.ned[1, idx],
                                 new_ned.item(0) - self.tree.ned[0, idx])
            new_pose = np.concatenate((new_ned, np.array([[new_chi]])), axis=0)
            tree_pose = np.concatenate((column(self.tree.ned, idx), np.array([[self.tree.course.item(idx)]])), axis=0)
            if self.collision(tree_pose, new_pose, map, radius) is False:
                self.tree.add(new_pose[0:3], Va, new_chi, cost, idx, 0)
                flag1=True
            # check to see if new node connects directly to end_node
            if (self.distance(new_pose, end_pose)<self.segment_length )\
                    and self.distance(new_pose, end_pose)>=2*radius\
                        and self.collision(new_pose, end_pose, map, radius) is False:
                flag = True
                self.tree.connect_to_goal[-1] = 1  # mark node as connecting to end.
            else:
                flag = False
        return flag

    def random_pose(self, map, pd):
        # generate a random pose
        pn = map.city_width * np.random.rand()
        pe = map.city_width * np.random.rand()
        chi = 0
        pose = np.array([[pn], [pe], [pd], [chi]])
        return pose

    def find_minimum_path(self, end_pose):
        # find the lowest cost path to the end node

        # find nodes that connect to end_node
        connecting_nodes = []
        for i in range(self.tree.num_waypoints):
            if self.tree.connect_to_goal.item(i)==1:
                connecting_nodes.append(i)
        # find minimum cost last node
        idx = np.argmin(self.tree.cost[connecting_nodes])
        # construct lowest cost path order
        path = [connecting_nodes[idx]]  # last node that connects to end node
        parent_node = self.tree.parent.item(connecting_nodes[idx])
        while parent_node >= 1:
            path.insert(0, int(parent_node))
            parent_node = self.tree.parent.item(int(parent_node))
        # construct waypoint path
        waypoints = msgWaypoints()
        for i in path:
            waypoints.add(column(self.tree.ned, i),
                          self.tree.airspeed.item(i),
                          self.tree.course.item(i),
                          np.inf,
                          np.inf,
                          np.inf)
        waypoints.add(end_pose[0:3],
                      self.tree.airspeed[-1],
                      end_pose.item(3),
                      np.inf,
                      np.inf,
                      np.inf)
        waypoints.type = self.tree.type
        return waypoints

    def smooth_path(self, waypoints, map, radius):
        # smooth the waypoint path
        smooth = [0]  # add the first waypoint
        ptr = 1
        while ptr <= waypoints.num_waypoints - 2:
            start_pose = np.concatenate((column(waypoints.ned, smooth[-1]), np.array([[waypoints.course[smooth[-1]]]])), axis=0)
            end_pose = np.concatenate((column(waypoints.ned, ptr+1), np.array([[waypoints.course[ptr+1]]])), axis=0)
            if self.collision(start_pose, end_pose, map, radius) is True\
                and self.distance(start_pose, end_pose) >= 2*radius:
                smooth.append(ptr)
            ptr += 1
        smooth.append(waypoints.num_waypoints-1)
        # construct smooth waypoint path
        smooth_waypoints = msgWaypoints()
        for i in smooth:
            smooth_waypoints.add(column(waypoints.ned, i),
                                 waypoints.airspeed.item(i),
                                 waypoints.course.item(i),
                                 np.inf,
                                 np.inf,
                                 np.inf)
        smooth_waypoints.type = waypoints.type
        return smooth_waypoints

    def plot_map(self, map, waypoints, smoothed_waypoints, radius):
        scale = 4000
        # initialize Qt gui application and window
        self.plot_app = pg.QtGui.QApplication([])  # initialize QT
        self.plot_window = gl.GLViewWidget()  # initialize the view object
        self.plot_window.setWindowTitle('World Viewer')
        self.plot_window.setGeometry(0, 0, 1500, 1500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(scale/20, scale/20, scale/20) # set the size of the grid (distance between each line)
        self.plot_window.addItem(grid) # add grid to viewer
        self.plot_window.setCameraPosition(distance=scale, elevation=50, azimuth=-90)
        self.plot_window.setBackgroundColor('k')  # set background color to black
        self.plot_window.show()  # display configured window
        self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        drawMap(map, self.plot_window)
        drawWaypoints(waypoints, radius, blue, self.plot_window)
        drawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        self.draw_tree(radius, green)
        # draw things to the screen
        self.plot_app.processEvents()

    def draw_tree(self, radius, color):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        Del = 0.05

        for i in range(1, self.tree.num_waypoints):
            parent = int(self.tree.parent.item(i))
            self.dubins_path.update(column(self.tree.ned, parent), self.tree.course[parent],
                                    column(self.tree.ned, i), self.tree.course[i], radius)
            points = self.points_along_path(Del)
            points = points @ R.T
            tree_color = np.tile(color, (points.shape[0], 1))
            tree_plot_object = gl.GLLinePlotItem(pos=points,
                                                 color=tree_color,
                                                 width=2,
                                                 antialias=True,
                                                 mode='line_strip')
            self.plot_window.addItem(tree_plot_object)


def mod(x):
    # force x to be between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col
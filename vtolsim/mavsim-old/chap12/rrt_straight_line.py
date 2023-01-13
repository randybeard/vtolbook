# rrt straight line path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - Brady Moon
#         4/11/2019 - RWB
import numpy as np
from message_types.msg_waypoints import msgWaypoints
from viewers.drawing import drawWaypoints, drawMap
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class rrtStraightLine():
    def __init__(self):
        self.segment_length = 300 # standard length of path segments
        self.tree = msgWaypoints()
        self.tree.type = 'fillet'
        self.plot_window = []
        self.plot_app = []

    def update(self, start_pose, end_pose, Va, map, radius):
        # add the start pose to the tree
        self.tree.add(start_pose, Va, np.inf, 0, 0, 0)
        # check to see if start_pose connects directly to end_pose
        if self.distance(start_pose, end_pose) < self.segment_length\
            and self.collision(start_pos, end_pos, map) == False:
            self.tree.add(end_pose, Va, np.inf, self.distance(start_pose, end_pose), 1, 1)
        else:
            num_paths = 0
            while num_paths < 3:
                flag = self.extend_tree(end_pose, Va, map)
                num_paths = num_paths + flag
        # find path with minimum cost to end_node
        waypoints_not_smooth = self.find_minimum_path(end_pose)
        waypoints = self.smooth_path(waypoints_not_smooth, map)
        self.plot_map(map, waypoints_not_smooth, waypoints, radius)
        return waypoints

    def distance(self, start_pose, end_pose):
        # compute distance between start and end pose
        d = np.linalg.norm(start_pose - end_pose)
        return d

    def collision(self, start_pose, end_pose, map):
        # check to see of path from start_pose to end_pose colliding with map
        collision_flag = False
        points = self.points_along_path(start_pose, end_pose, 100)
        for i in range(points.shape[1]):
            if self.height_above_ground(map, column(points, i)) <= 0:
                collision_flag = True
        return collision_flag

    def points_along_path(self, start_pose, end_pose, N):
        # returns points along path separated by Del
        points = start_pose
        q = (end_pose - start_pose)
        L = np.linalg.norm(q)
        q = q / L
        w = start_pose
        for i in range(1, N):
            w = w + (L / N) * q
            points = np.append(points, w, axis=1)
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

    def extend_tree(self, end_pose, Va, map):
        # extend tree by randomly selecting pose and extending tree toward that pose
        flag1 = False
        while flag1==False:
            random_pose = self.random_pose(map, end_pose.item(2))
            # find leaf on tree that is closest to random_pose
            tmp = self.tree.ned-np.tile(random_pose, (1, self.tree.num_waypoints))
            tmp1 = np.diag(tmp.T @ tmp)
            idx = np.argmin(tmp1)
            dist = tmp1.item(idx)
            L = np.min([np.sqrt(dist), self.segment_length])
            cost = self.tree.cost.item(idx) + L
            tmp = random_pose-column(self.tree.ned,idx)
            new_pose = column(self.tree.ned,idx) + L * (tmp / np.linalg.norm(tmp))

            if self.collision(column(self.tree.ned, idx), new_pose, map)==False:
                self.tree.add(new_pose, Va, np.inf, cost, idx, 0)
                flag1=True
            # check to see if new node connects directly to end_node
            if (self.distance(new_pose, end_pose)<self.segment_length )\
                    and (self.collision(new_pose, end_pose, map)==False):
                flag = True
                self.tree.connect_to_goal[-1] = True  # mark node as connecting to end.
            else:
                flag = False
        return flag

    def random_pose(self, map, pd):
        # generate a random pose
        pn = map.city_width * np.random.rand()
        pe = map.city_width * np.random.rand()
        pose = np.array([[pn], [pe], [pd]])
        return pose

    def find_minimum_path(self, end_pose):
        # find the lowest cost path to the end node

        # find nodes that connect to end_node
        connecting_nodes = []
        for i in range(self.tree.num_waypoints):
            if self.tree.connect_to_goal.item(i)==True:
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
                          np.inf,
                          np.inf,
                          np.inf,
                          np.inf)
        waypoints.add(end_pose,
                      self.tree.airspeed[-1],
                      np.inf,
                      np.inf,
                      np.inf,
                      np.inf)
        waypoints.type = self.tree.type
        return waypoints

    def smooth_path(self, waypoints, map):
        # smooth the waypoint path
        smooth = [0]  # add the first waypoint
        ptr = 1
        while ptr <= waypoints.num_waypoints - 2:
            pose_s = column(waypoints.ned, smooth[-1])
            pose_e = column(waypoints.ned, ptr+1)
            if (self.collision(pose_s, pose_e, map) == True):
                smooth.append(ptr)
            ptr += 1
        smooth.append(waypoints.num_waypoints-1)
        # construct smooth waypoint path
        smooth_waypoints = msgWaypoints()
        for i in smooth:
            smooth_waypoints.add(column(waypoints.ned, i),
                                 waypoints.airspeed.item(i),
                                 np.inf,
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
        draw_tree(self.tree, radius, green, self.plot_window)
        # draw things to the screen
        self.plot_app.processEvents()


def draw_tree(tree, radius, color, window):
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = R @ tree.ned
    for i in range(points.shape[1]):
        line_color = np.tile(color, (2, 1))
        parent = int(tree.parent.item(i))
        line_pts = np.concatenate((column(points, i).T, column(points, parent).T), axis=0)
        line = gl.GLLinePlotItem(pos=line_pts,
                                 color=line_color,
                                 width=2,
                                 antialias=True,
                                 mode='line_strip')
        window.addItem(line)


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col
"""
vtolsim: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/1/2019 - Randy Beard
        4/15/2019 - BGM
        5/3/2019 - Randy Beard
"""

import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import Euler2Rotation
#from chap11.dubins_parameters import dubinsParameters

class drawVtol():
    def __init__(self, state, window):
        """
        Draw the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.pn  # north position
            state.pe  # east position
            state.h   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        self.unit_length = .1
        # get points that define the non-rotated, non-translated vtol and the mesh colors
        self.vtol_points, self.vtol_meshColors = self.get_fuselage_points()
        self.motor_points, self.motor_meshColors = self.get_motor_points()
        self.rotor_points, self.rotor_colors = self.get_rotor_points()
        vtol_position = np.array([[state.pn], [state.pe], [-state.h]])  # NED coordinates
        # attitude of vtol as a rotation matrix R from body to inertial
        R_bi = Euler2Rotation(state.phi, state.theta, state.psi)
        # convert North-East Down to East-North-Up for rendering
        R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # fuselage and wings
        rotated_body = self.rotate_points(self.vtol_points, R_bi)
        translated_body = self.translate_points(rotated_body, vtol_position)
        translated_body = R_ned @ translated_body
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        body_mesh = self.fuselage_points_to_mesh(translated_body)
        self.vtol_body = gl.GLMeshItem(vertexes=body_mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.vtol_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        window.addItem(self.vtol_body)  # add body to plot

        # right motor
        self.rmotor_pos = np.array([[-2.5 * self.unit_length],
                                    [2.0 * self.unit_length],
                                    [0]])
        R_rmotor = Euler2Rotation(0.0, state.right_rotor, 0.0)
        rotated_rmotor = self.rotate_points(self.motor_points, R_bi @ R_rmotor)
        rmotor_position = vtol_position + R_bi @ self.rmotor_pos
        translated_rmotor = self.translate_points(rotated_rmotor, rmotor_position)
        translated_rmotor = R_ned @ translated_rmotor
        rmotor_mesh = self.motor_points_to_mesh(translated_rmotor)
        self.vtol_rmotor = gl.GLMeshItem(vertexes=rmotor_mesh,
                                         vertexColors=self.motor_meshColors,
                                         drawEdges=True,
                                         smooth=False,
                                         computeNormals=False)
        window.addItem(self.vtol_rmotor)

        # left motor
        self.lmotor_pos = np.array([[-2.5 * self.unit_length],
                                    [-2.0 * self.unit_length],
                                    [0]])
        R_lmotor = Euler2Rotation(0.0, state.left_rotor, 0.0)
        rotated_lmotor = self.rotate_points(self.motor_points, R_bi @ R_lmotor)
        lmotor_position = vtol_position + R_bi @ self.lmotor_pos
        translated_lmotor = self.translate_points(rotated_lmotor, lmotor_position)
        translated_lmotor = R_ned @ translated_lmotor
        lmotor_mesh = self.motor_points_to_mesh(translated_lmotor)
        self.vtol_lmotor = gl.GLMeshItem(vertexes=lmotor_mesh,
                                         vertexColors=self.motor_meshColors,
                                         drawEdges=True,
                                         smooth=False,
                                         computeNormals=False)
        window.addItem(self.vtol_lmotor)

        # back rotor
        self.back_rotor_pos = np.array([[-4.5 * self.unit_length], [0], [0]])
        rotated_back_rotor = self.rotate_points(self.rotor_points, R_bi)
        back_rotor_position = vtol_position + R_bi @ self.back_rotor_pos
        translated_back_rotor = self.translate_points(rotated_back_rotor, back_rotor_position)
        translated_back_rotor = R_ned @ translated_back_rotor
        self.back_rotor =  gl.GLLinePlotItem(pos=translated_back_rotor.T,
                                             color=self.rotor_colors,
                                             width=2,
                                             antialias=True,
                                             mode='line_strip')
        window.addItem(self.back_rotor)

        # right rotor
        self.rotor_pos = np.array([[1.1 * self.unit_length],
                                   [0],
                                   [0]]) # with respect to motor
        self.R_rotor = Euler2Rotation(0.0, np.pi / 2, 0.0)
        rotated_right_rotor = self.rotate_points(self.rotor_points, R_bi @ R_rmotor @ self.R_rotor)
        right_rotor_position = vtol_position \
                               + R_bi @ self.rmotor_pos \
                               + R_bi @ R_rmotor @ self.rotor_pos
        translated_right_rotor = self.translate_points(rotated_right_rotor, right_rotor_position)
        translated_right_rotor = R_ned @ translated_right_rotor
        self.right_rotor =  gl.GLLinePlotItem(pos=translated_right_rotor.T,
                                              color=self.rotor_colors,
                                              width=2,
                                              antialias=True,
                                              mode='line_strip')
        window.addItem(self.right_rotor)

        # left rotor
        rotated_left_rotor = self.rotate_points(self.rotor_points, R_bi @ R_lmotor @ self.R_rotor)
        left_rotor_position = vtol_position \
                               + R_bi @ self.rmotor_pos \
                               + R_bi @ R_lmotor @ self.rotor_pos
        translated_left_rotor = self.translate_points(rotated_left_rotor, left_rotor_position)
        translated_left_rotor = R_ned @ translated_left_rotor
        self.left_rotor =  gl.GLLinePlotItem(pos=translated_left_rotor.T,
                                             color=self.rotor_colors,
                                             width=2,
                                             antialias=True,
                                             mode='line_strip')
        window.addItem(self.left_rotor)

    def update(self, state):
        # NED coordinates of vtol
        vtol_position = np.array([[state.pn], [state.pe], [-state.h]])
        # attitude of vtol as a rotation matrix R from body to inertial
        R_bi = Euler2Rotation(state.phi, state.theta, state.psi)
        # convert North-East Down to East-North-Up for rendering
        R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # fuselage and wings
        rotated_body = self.rotate_points(self.vtol_points, R_bi)
        translated_body = self.translate_points(rotated_body, vtol_position)
        translated_body = R_ned @ translated_body
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        body_mesh = self.fuselage_points_to_mesh(translated_body)
        # draw VTOL by resetting mesh using rotated and translated points
        self.vtol_body.setMeshData(vertexes=body_mesh, vertexColors=self.vtol_meshColors)

        # right motor
        R_rmotor =  Euler2Rotation(0.0, state.right_rotor, 0.0)
        rotated_rmotor = self.rotate_points(self.motor_points, R_bi @ R_rmotor)
        rmotor_position = vtol_position + R_bi @ self.rmotor_pos
        translated_rmotor = self.translate_points(rotated_rmotor, rmotor_position)
        translated_rmotor = R_ned @ translated_rmotor
        rmotor_mesh = self.motor_points_to_mesh(translated_rmotor)
        self.vtol_rmotor.setMeshData(vertexes=rmotor_mesh,
                                     vertexColors=self.motor_meshColors)
        # left motor
        R_lmotor = Euler2Rotation(0.0, state.left_rotor, 0.0)
        rotated_lmotor = self.rotate_points(self.motor_points, R_bi @ R_lmotor)
        lmotor_position = vtol_position + R_bi @ self.lmotor_pos
        translated_lmotor = self.translate_points(rotated_lmotor, lmotor_position)
        translated_lmotor = R_ned @ translated_lmotor
        lmotor_mesh = self.motor_points_to_mesh(translated_lmotor)
        self.vtol_lmotor.setMeshData(vertexes=lmotor_mesh,
                                     vertexColors=self.motor_meshColors)

        # back rotor
        rotated_back_rotor = self.rotate_points(self.rotor_points, R_bi)
        back_rotor_position = vtol_position + R_bi @ self.back_rotor_pos
        translated_back_rotor = self.translate_points(rotated_back_rotor, back_rotor_position)
        translated_back_rotor = R_ned @ translated_back_rotor
        self.back_rotor.setData(pos=translated_back_rotor.T)

        # right rotor
        rotated_right_rotor = self.rotate_points(self.rotor_points, R_bi @ R_rmotor @ self.R_rotor)
        right_rotor_position = vtol_position \
                               + R_bi @ self.rmotor_pos \
                               + R_bi @ R_rmotor @ self.rotor_pos
        translated_right_rotor = self.translate_points(rotated_right_rotor, right_rotor_position)
        translated_right_rotor = R_ned @ translated_right_rotor
        self.right_rotor.setData(pos=translated_right_rotor.T)

        # left rotor
        rotated_left_rotor = self.rotate_points(self.rotor_points, R_bi @ R_lmotor @ self.R_rotor)
        left_rotor_position = vtol_position \
                               + R_bi @ self.lmotor_pos \
                               + R_bi @ R_lmotor @ self.rotor_pos
        translated_left_rotor = self.translate_points(rotated_left_rotor, left_rotor_position)
        translated_left_rotor = R_ned @ translated_left_rotor
        self.left_rotor.setData(pos=translated_left_rotor.T)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def get_fuselage_points(self):
        """"
            Points that define the vtol, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define MAV body parameters
        fuse_h = 1.0 * self.unit_length
        fuse_w = 1.0 * self.unit_length
        fuse_l1 = 3.0 * self.unit_length
        fuse_l2 = 1.0 * self.unit_length
        fuse_l3 = -3.0 * self.unit_length
        wing_l1 = 1.0 * self.unit_length
        wing_l2 = -4.0 * self.unit_length
        wing_l3 = -5.0 * self.unit_length
        wing_l4 = -3.5 * self.unit_length
        wing_w1 = 6.0 * self.unit_length
        wing_w2 = 2.0 * self.unit_length
        winglet_d = 1.0 * self.unit_length
        tail_h = 2.0 * self.unit_length
        tail_l = -2.0 * self.unit_length
        tail_pos_l = -4.5 * self.unit_length
        tail_pos_w = 1.5 * self.unit_length

        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        points = np.array([[fuse_l1, 0, 0],  # point [0] - nose-tip
                           [fuse_l2, fuse_w / 2.0, -fuse_h / 2.0],  # point [1] - nose cone
                           [fuse_l2, -fuse_w / 2.0, -fuse_h / 2.0],  # point [2] - nose cone
                           [fuse_l2, -fuse_w / 2.0, fuse_h / 2.0],  # point [3] - nose cone
                           [fuse_l2, fuse_w / 2.0, fuse_h / 2.0],  # point [4] - nose cone
                           [fuse_l3, 0, 0],  # point [5] - end of fuselage
                           [wing_l1, 0, 0],  # point [6] wing
                           [wing_l2, -wing_w1/2, 0], # point [7] wing
                           [wing_l3, -wing_w1/2, 0],  # point [8] wing
                           [wing_l3, -wing_w2/2, 0],  # point [9] wing
                           [(wing_l4+wing_l3)/2, -wing_w2/2, 0],  # point [10] wing
                           [wing_l4, -wing_w2/6, 0],  # point [11] wing
                           [wing_l4, wing_w2/6, 0],  # point [12] wing
                           [(wing_l4+wing_l3)/2, wing_w2/2, 0],  # point [13] wing
                           [wing_l3, wing_w2/2, 0],  # point [14] wing
                           [wing_l3, wing_w1/2, 0],  # point [15] wing
                           [wing_l2, wing_w1/2, 0], # point [16] wing
                           [(wing_l2+wing_l3)/2, -wing_w1/2, winglet_d], # point [17] left winglet
                           [wing_l3, -wing_w1/2, winglet_d],  # point [18] left winglet
                           [(wing_l2+wing_l3)/2, wing_w1/2, winglet_d], # point [19] right winglet
                           [wing_l3, wing_w1/2, winglet_d],  # point [20] right winglet
                           [tail_pos_l + tail_l, -tail_pos_w, 0],  # point [21] left tail
                           [tail_pos_l + tail_l, -tail_pos_w, -tail_h],  # point [22] left tail
                           [tail_pos_l + tail_l/2, -tail_pos_w, -tail_h],  # point [23] left tail
                           [tail_pos_l, -tail_pos_w, 0],  # point [24] left tail
                           [tail_pos_l + tail_l, tail_pos_w, 0],  # point [25] right tail
                           [tail_pos_l + tail_l, tail_pos_w, -tail_h],  # point [26] right tail
                           [tail_pos_l + tail_l/2, tail_pos_w, -tail_h],  # point [27] right tail
                           [tail_pos_l, tail_pos_w, 0],  # point [28] right tail
                           ]).T

        #   define the colors for each face of triangular mesh
        #red = np.array([1., 0., 0., 1])
        red = np.array([211, 68, 63, 256])/256
        #green = np.array([0., 1., 0., 1])
        green = np.array([63, 211, 105, 256])/256.
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((25, 3, 4), dtype=np.float32)
        meshColors[0] = blue  # nose-top
        meshColors[1] = blue  # nose-right
        meshColors[2] = red  # nose-bottom
        meshColors[3] = blue  # nose-left
        meshColors[4] = blue  # fuselage-left
        meshColors[5] = blue  # fuselage-top
        meshColors[6] = blue  # fuselage-right
        meshColors[7] = red  # fuselage-bottom
        meshColors[8] = green  # wing
        meshColors[9] = green  # wing
        meshColors[10] = green  # wing
        meshColors[11] = green  # wing
        meshColors[12] = green  # wing
        meshColors[13] = green  # wing
        meshColors[14] = green  # wing
        meshColors[15] = green  # wing
        meshColors[16] = green  # wing
        meshColors[17] = green  # winglet
        meshColors[18] = green  # winglet
        meshColors[19] = green  # winglet
        meshColors[20] = green  # winglet
        meshColors[21] = blue  # tail
        meshColors[22] = blue  # tail
        meshColors[23] = blue  # tail
        meshColors[24] = blue  # tail
        return points, meshColors

    def fuselage_points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],  # nose-top
                         [points[0], points[1], points[4]],  # nose-right
                         [points[0], points[3], points[4]],  # nose-bottom
                         [points[0], points[3], points[2]],  # nose-left
                         [points[5], points[2], points[3]],  # fuselage-left
                         [points[5], points[1], points[2]],  # fuselage-top
                         [points[5], points[1], points[4]],  # fuselage-right
                         [points[5], points[3], points[4]],  # fuselage-bottom
                         [points[6], points[7], points[11]],  # wing
                         [points[7], points[11], points[10]],  # wing
                         [points[7], points[10], points[9]],  # wing
                         [points[7], points[8], points[9]],  # wing
                         [points[6], points[11], points[12]],  # wing
                         [points[6], points[12], points[16]],  # wing
                         [points[12], points[13], points[16]],  # wing
                         [points[13], points[16], points[14]],  # wing
                         [points[14], points[15], points[16]],  # wing
                         [points[7], points[8], points[17]],  # winglet
                         [points[17], points[18], points[8]],  # winglet
                         [points[15], points[16], points[19]],  # winglet
                         [points[15], points[19], points[20]],  # winglet
                         [points[21], points[22], points[23]],  # tail
                         [points[21], points[23], points[24]],  # tail
                         [points[25], points[26], points[27]],  # tail
                         [points[25], points[27], points[28]],  # tail
                         ])
        return mesh

    def get_motor_points(self):
        """"
            Points that define the motor
        """
        # define MAV body parameters
        height = 0.5 * self.unit_length
        width = 0.5 * self.unit_length
        length = 1.0 * self.unit_length

        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        points = np.array([[0, width/2, height/2],  # [0]
                           [0, width/2, -height/2],  # [1]
                           [0, -width/2, -height/2],  # [2]
                           [0, -width/2, height/2],  # [3]
                           [length, width/2, height/2],  # [4]
                           [length, width/2, -height/2],  # [5]
                           [length, -width/2, -height/2],  # [6]
                           [length, -width/2, height/2],  # [7]
                           ]).T

        #   define the colors for each face of triangular mesh
        #red = np.array([1., 0., 0., 1])
        red = np.array([211, 68, 63, 256])/256
        green = np.array([0., 1., 0., 1])
        blue = np.array([66, 161, 244, 256])/256.
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = blue  # end
        meshColors[1] = blue  # end
        meshColors[2] = blue # top
        meshColors[3] = blue  # top
        meshColors[4] = blue  # right
        meshColors[5] = blue  # right
        meshColors[6] = red  # bottom
        meshColors[7] = red  # bottom
        meshColors[8] = blue  # left
        meshColors[9] = blue  # left
        meshColors[10] = red  # end
        meshColors[11] = red  # end
        return points, meshColors

    def motor_points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],  # end
                         [points[0], points[3], points[2]],  # end
                         [points[1], points[5], points[6]],  # top
                         [points[1], points[2], points[6]],  # top
                         [points[0], points[1], points[5]],  # right
                         [points[0], points[4], points[5]],  # right
                         [points[0], points[4], points[7]],  # bottom
                         [points[0], points[3], points[7]],  # bottom
                         [points[3], points[7], points[6]],  # left
                         [points[3], points[2], points[6]],  # left
                         [points[4], points[5], points[6]],  # end
                         [points[4], points[7], points[6]],  # end
                         ])
        return mesh

    def get_rotor_points(self):
        radius = 0.6 * self.unit_length
        N = 10
        theta = 0
        theta_list = [theta]
        while theta < 2*np.pi:
            theta += 0.1
            theta_list.append(theta)
        points = np.array([[radius, 0, 0]])
        for angle in theta_list:
            new_point = np.array([[radius * np.cos(angle),
                                   radius * np.sin(angle),
                                   0]])
            points = np.concatenate((points, new_point), axis=0)
        color = np.array([1., 1., 0., 1])
        path_color = np.tile(color, (points.shape[0], 1))
        return points.T, path_color

class drawPath():
    def __init__(self, path, color, window):
        self.color = color
        if path.type == 'line':
            scale = 1000
            points = self.straight_line_points(path, scale)
        elif path.type == 'orbit':
            points = self.orbit_points(path)
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object =  gl.GLLinePlotItem(pos=points,
                                                   color=path_color,
                                                   width=2,
                                                   antialias=True,
                                                   mode='line_strip')
        window.addItem(self.path_plot_object)

    def update(self, path):
        if path.type == 'line':
            scale = 1000
            points = self.straight_line_points(path, scale)
        elif path.type == 'orbit':
            points = self.orbit_points(path)
        self.path_plot_object.setData(pos=points)

    def straight_line_points(self, path, scale):
        points = np.array([[path.line_origin.item(0),
                            path.line_origin.item(1),
                            path.line_origin.item(2)],
                           [path.line_origin.item(0) + scale * path.line_direction.item(0),
                            path.line_origin.item(1) + scale * path.line_direction.item(1),
                            path.line_origin.item(2) + scale * path.line_direction.item(2)]])
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        return points


    def orbit_points(self, path):
        N = 10
        theta = 0
        theta_list = [theta]
        while theta < 2*np.pi:
            theta += 0.1
            theta_list.append(theta)
        points = np.array([[path.orbit_center.item(0) + path.orbit_radius,
                            path.orbit_center.item(1),
                            path.orbit_center.item(2)]])
        for angle in theta_list:
            new_point = np.array([[path.orbit_center.item(0) + path.orbit_radius * np.cos(angle),
                                   path.orbit_center.item(1) + path.orbit_radius * np.sin(angle),
                                   path.orbit_center.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        return points


class drawWaypoints():
    def __init__(self, waypoints, radius, color, window):
        self.radius = radius
        self.color = color
        if waypoints.type=='straight_line' or waypoints.type=='fillet':
            points = self.straight_waypoint_points(waypoints)
        elif waypoints.type=='dubins':
            points = self.dubins_points(waypoints, self.radius, 0.1)
        waypoint_color = np.tile(color, (points.shape[0], 1))
        self.waypoint_plot_object = gl.GLLinePlotItem(pos=points,
                                                      color=waypoint_color,
                                                      width=2,
                                                      antialias=True,
                                                      mode='line_strip')
        window.addItem(self.waypoint_plot_object)

    def update(self, waypoints):
        if waypoints.type=='straight_line' or waypoints.type=='fillet':
            points = self.straight_waypoint_points(waypoints)
        elif waypoints.type=='dubins':
            points = self.dubins_points(waypoints, self.radius, 0.1)
        self.waypoint_plot_object.setData(pos=points)

    def straight_waypoint_points(self, waypoints):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(waypoints.ned)
        return points.T

    def dubins_points(self, waypoints, radius, Del):
        # returns a list of points along the dubins path
        initialize_points = True
        dubins_path = dubinsParameters()
        for j in range(0, waypoints.num_waypoints-1):
            dubins_path.update(
                waypoints.ned[:, j:j+1],
                waypoints.course.item(j),
                waypoints.ned[:, j+1:j+2],
                waypoints.course.item(j+1),
                radius)

            # points along start circle
            th1 = np.arctan2(dubins_path.p_s.item(1) - dubins_path.center_s.item(1),
                             dubins_path.p_s.item(0) - dubins_path.center_s.item(0))
            th1 = self.mod(th1)
            th2 = np.arctan2(dubins_path.r1.item(1) - dubins_path.center_s.item(1),
                             dubins_path.r1.item(0) - dubins_path.center_s.item(0))
            th2 = self.mod(th2)
            th = th1
            theta_list = [th]
            if dubins_path.dir_s > 0:
                if th1 >= th2:
                    while th < th2 + 2*np.pi:
                        th += Del
                        theta_list.append(th)
                else:
                    while th < th2:
                        th += Del
                        theta_list.append(th)
            else:
                if th1 <= th2:
                    while th > th2 - 2*np.pi:
                        th -= Del
                        theta_list.append(th)
                else:
                    while th > th2:
                        th -= Del
                        theta_list.append(th)

            if initialize_points:
                points = np.array([[dubins_path.center_s.item(0) + dubins_path.radius * np.cos(theta_list[0]),
                                    dubins_path.center_s.item(1) + dubins_path.radius * np.sin(theta_list[0]),
                                    dubins_path.center_s.item(2)]])
                initialize_points = False
            for angle in theta_list:
                new_point = np.array([[dubins_path.center_s.item(0) + dubins_path.radius * np.cos(angle),
                                       dubins_path.center_s.item(1) + dubins_path.radius * np.sin(angle),
                                       dubins_path.center_s.item(2)]])
                points = np.concatenate((points, new_point), axis=0)

            # points along straight line
            sig = 0
            while sig <= 1:
                new_point = np.array([[(1 - sig) * dubins_path.r1.item(0) + sig * dubins_path.r2.item(0),
                                       (1 - sig) * dubins_path.r1.item(1) + sig * dubins_path.r2.item(1),
                                       (1 - sig) * dubins_path.r1.item(2) + sig * dubins_path.r2.item(2)]])
                points = np.concatenate((points, new_point), axis=0)
                sig += Del

            # points along end circle
            th2 = np.arctan2(dubins_path.p_e.item(1) - dubins_path.center_e.item(1),
                             dubins_path.p_e.item(0) - dubins_path.center_e.item(0))
            th2 = self.mod(th2)
            th1 = np.arctan2(dubins_path.r2.item(1) - dubins_path.center_e.item(1),
                             dubins_path.r2.item(0) - dubins_path.center_e.item(0))
            th1 = self.mod(th1)
            th = th1
            theta_list = [th]
            if dubins_path.dir_e > 0:
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
                new_point = np.array([[dubins_path.center_e.item(0) + dubins_path.radius * np.cos(angle),
                                       dubins_path.center_e.item(1) + dubins_path.radius * np.sin(angle),
                                       dubins_path.center_e.item(2)]])
                points = np.concatenate((points, new_point), axis=0)

        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        return points

    def mod(self, x):
        # force x to be between 0 and 2*pi
        while x < 0:
            x += 2*np.pi
        while x > 2*np.pi:
            x -= 2*np.pi
        return x

class drawMap():
    def __init__(self, map, window):
        self.window = window
        # draw map of the world: buildings
        fullMesh = np.array([], dtype=np.float32).reshape(0, 3, 3)
        fullMeshColors = np.array([], dtype=np.float32).reshape(0, 3, 4)
        for i in range(0, map.num_city_blocks):
            for j in range(0, map.num_city_blocks):
                mesh, meshColors = self.building_vert_face(map.building_north[0, i],
                                                           map.building_east[0, j],
                                                           map.building_width,
                                                           map.building_height[i, j])
                fullMesh = np.concatenate((fullMesh, mesh), axis=0)
                fullMeshColors = np.concatenate((fullMeshColors, meshColors), axis=0)
        self.ground_mesh = gl.GLMeshItem(
            vertexes=fullMesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=fullMeshColors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        self.window.addItem(self.ground_mesh)

    def update(self, map):
        # draw map of the world: buildings
        fullMesh = np.array([], dtype=np.float32).reshape(0, 3, 3)
        fullMeshColors = np.array([], dtype=np.float32).reshape(0, 3, 4)
        for i in range(0, map.num_city_blocks):
            for j in range(0, map.num_city_blocks):
                mesh, meshColors = self.building_vert_face(map.building_north[0, i],
                                                           map.building_east[0, j],
                                                           map.building_width,
                                                           map.building_height[i, j])
                fullMesh = np.concatenate((fullMesh, mesh), axis=0)
                fullMeshColors = np.concatenate((fullMeshColors, meshColors), axis=0)
        self.ground_mesh.setData(vertexes=fullMesh, vertexColors=fullMeshColors)

    def building_vert_face(self, n, e, width, height):
        # define patches for a building located at (x, y)
        # vertices of the building
        points = np.array([[e + width / 2, n + width / 2, 0],  # NE 0
                           [e + width / 2, n - width / 2, 0],  # SE 1
                           [e - width / 2, n - width / 2, 0],  # SW 2
                           [e - width / 2, n + width / 2, 0],  # NW 3
                           [e + width / 2, n + width / 2, height],  # NE Higher 4
                           [e + width / 2, n - width / 2, height],  # SE Higher 5
                           [e - width / 2, n - width / 2, height],  # SW Higher 6
                           [e - width / 2, n + width / 2, height]])  # NW Higher 7
        mesh = np.array([[points[0], points[3], points[4]],  # North Wall
                         [points[7], points[3], points[4]],  # North Wall
                         [points[0], points[1], points[5]],  # East Wall
                         [points[0], points[4], points[5]],  # East Wall
                         [points[1], points[2], points[6]],  # South Wall
                         [points[1], points[5], points[6]],  # South Wall
                         [points[3], points[2], points[6]],  # West Wall
                         [points[3], points[7], points[6]],  # West Wall
                         [points[4], points[7], points[5]],  # Top
                         [points[7], points[5], points[6]]])  # Top

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((10, 3, 4), dtype=np.float32)
        meshColors[0] = green
        meshColors[1] = green
        meshColors[2] = green
        meshColors[3] = green
        meshColors[4] = green
        meshColors[5] = green
        meshColors[6] = green
        meshColors[7] = green
        meshColors[8] = yellow
        meshColors[9] = yellow
        return mesh, meshColors

class drawTrajectory:
    def __init__(self, points, color, window):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(points)
        
        points = points.T
        self.color = color
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object =  gl.GLLinePlotItem(pos=points,
                                                   color=path_color,
                                                   width=2,
                                                   antialias=True,
                                                   mode='line_strip')
        window.addItem(self.path_plot_object)

    def update(self, points):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(points)
        self.path_plot_object.setData(pos=points)

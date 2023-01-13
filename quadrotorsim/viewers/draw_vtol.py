"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
        3/2/2022
"""

import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation

class DrawVtol():
    def __init__(self, state, window):
        """
        Draw the winged vtol

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.p  # inertial position (NED)
            state.R  # rotation R_b^i
        """
        self.unit_length = 0.5
        vtol_position = state.pos  # NED coordinates
        R_bi = state.rot  # body to inertial rotation
        # convert North-East Down to East-North-Up for rendering
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # colors
        self.mygrey1 = np.array([0.8, 0.8, 0.8, 1])  # light
        self.mygrey2 = np.array([0.6, 0.6, 0.6, 1])
        self.mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        self.mygrey4 = np.array([0.3, 0.3, 0.3, 1])  # dark

        # body
        self.body_forward_x = 15 * self.unit_length
        self.body_backward_x = 27 * self.unit_length
        self.body_y1 = 2 * self.unit_length
        self.body_y2 = 3 * self.unit_length
        self.body_y3 = 1 * self.unit_length
        self.body_z1 = 0 * self.unit_length
        self.body_z2 = 2 * self.unit_length
        self.body_z3 = 4 * self.unit_length
        self.body_z4 = 0 * self.unit_length
        self.wing_span = 60 * self.unit_length
        self.wing_chord = 8 * self.unit_length
        self.wing_depth = 2 * self.unit_length
        self.wing_position = np.array([10., 0., 0.]) * self.unit_length
        self.htail_span = self.wing_span / 3
        self.htail_chord = self.wing_chord / 2
        self.htail_depth = self.wing_depth / 3
        self.htail_position = np.array([-self.body_backward_x + self.htail_chord, 0, 0])
        self.vtail_span = self.htail_span / 3
        self.vtail_chord = self.wing_chord / 1.5
        self.vtail_depth = self.wing_depth / 2
        self.vtail_position = np.array([-self.body_backward_x + self.vtail_chord, 0, 0])
        self.body_points, self.body_index, self.body_meshColors = self.get_body_points()
        self.vtol_body = self.add_object(
            self.body_points,
            self.body_index,
            self.body_meshColors,
            R_bi,
            vtol_position)
        window.addItem(self.vtol_body)  # add body to plot

        # arms
        self.arm_w = 2. * self.unit_length # width of arm
        self.arm_l = 12. * self.unit_length  # length of arm
        arm_front_x = self.wing_position.item(0)-self.wing_chord/2  # x position of front arm
        arm_front_y = self.wing_span/4  # y position of front arm
        arm_front_z = 1 * self.unit_length  # z position of front arm
        arm_back_x = self.wing_position.item(0)-self.wing_chord/2  # x position of back arm
        arm_back_y = self.wing_span/4  # y position of back arm
        arm_back_z = 1 * self.unit_length  # z position of back arm
        self.arm_points, self.arm_index, self.arm_meshColors = self.get_arm_points()
        # arm 1
        self.R_arm1 = euler_to_rotation(0.0, 0.0, 0.0)
        self.arm1_position = np.array([[arm_front_x, arm_front_y, arm_front_z]]).T
        self.arm1 = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm1,
            vtol_position + R_bi @ self.arm1_position)
        window.addItem(self.arm1)
        # arm 2
        self.R_arm2 = euler_to_rotation(0.0, 0.0, 0.0)
        self.arm2_position = np.array([[arm_front_x, -arm_front_y, arm_front_z]]).T
        self.arm2 = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm2,
            vtol_position + R_bi @ self.arm2_position)
        window.addItem(self.arm2)
        # arm 3
        self.R_arm3 = euler_to_rotation(0.0, 0.0, np.pi)
        self.arm3_position = np.array([[arm_back_x, arm_back_y, arm_back_z]]).T
        self.arm3 = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm3,
            vtol_position + R_bi @ self.arm3_position)
        window.addItem(self.arm3)
        # arm 4
        self.R_arm4 = euler_to_rotation(0.0, 0.0, np.pi)
        self.arm4_position = np.array([[arm_back_x, -arm_back_y, arm_back_z]]).T
        self.arm4 = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm4,
            vtol_position + R_bi @ self.arm4_position)
        window.addItem(self.arm4)

        # motors
        scale = 1.1
        self.motor_points, self.motor_index, self.motor_meshColors = self.get_motor_points()
        self.R_motor1 = euler_to_rotation(0.0, np.pi/2, np.pi/4)
        self.motor1_position = self.arm1_position + self.R_arm1 @ np.array([[scale*self.arm_l, 0, 0]]).T
        self.motor1 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor1,
            vtol_position + R_bi @ self.motor1_position)
        window.addItem(self.motor1)
        self.R_motor2 = euler_to_rotation(0.0, np.pi/2, -np.pi/4)
        self.motor2_position = self.arm2_position + self.R_arm2 @ np.array([[scale*self.arm_l, 0, 0]]).T
        self.motor2 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor2,
            vtol_position + R_bi @ self.motor2_position)
        window.addItem(self.motor2)
        self.R_motor3 = euler_to_rotation(0.0, np.pi/2, 3*np.pi/4)
        self.motor3_position = self.arm3_position + self.R_arm3 @ np.array([[self.arm_l, 0, 0]]).T
        self.motor3 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor3,
            vtol_position + R_bi @ self.motor3_position)
        window.addItem(self.motor3)
        self.R_motor4 = euler_to_rotation(0.0, np.pi/2, -3*np.pi/4)
        self.motor4_position = self.arm4_position + self.R_arm4 @ np.array([[self.arm_l, 0, 0]]).T
        self.motor4 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor4,
            vtol_position + R_bi @ self.motor4_position)
        window.addItem(self.motor4)

        # rotors
        self.rotor_points, self.rotor_index, self.rotor_meshColors = self.get_rotor_points()
        self.R_rotor1 = euler_to_rotation(0.0, 0.0, 0.0)
        #self.rotor1_position = self.arm1_position + self.R_arm1 @ np.array([[scale*self.arm_l, 0, -self.arm_w]]).T
        self.rotor1_position = self.arm1_position + self.R_arm1 @ np.array([[scale * self.arm_l, 0, 0]]).T
        self.rotor1 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor1,
            vtol_position + R_bi @ self.rotor1_position)
        window.addItem(self.rotor1)
        self.R_rotor2 = euler_to_rotation(0.0, 0.0, 0.0)
        #self.rotor2_position = self.arm2_position + self.R_arm2 @ np.array([[scale*self.arm_l, 0, -self.arm_w]]).T
        self.rotor2_position = self.arm2_position + self.R_arm2 @ np.array([[scale*self.arm_l, 0, 0]]).T
        self.rotor2 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor2,
            vtol_position + R_bi @ self.rotor2_position)
        window.addItem(self.rotor2)
        self.R_rotor3 = euler_to_rotation(0.0, 0.0, 0.0)
        self.rotor3_position = self.arm3_position + self.R_arm3 @ np.array([[self.arm_l, 0, -self.arm_w]]).T
        self.rotor3 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor3,
            vtol_position + R_bi @ self.rotor3_position)
        window.addItem(self.rotor3)
        self.R_rotor4 = euler_to_rotation(0.0, 0.0, 0.0)
        self.rotor4_position = self.arm4_position + self.R_arm4 @ np.array([[self.arm_l, 0, -self.arm_w]]).T
        self.rotor4 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor4,
            vtol_position + R_bi @ self.rotor4_position)
        window.addItem(self.rotor4)

    def update(self, state):
        # NED coordinates of vtol
        vtol_position = state.pos
        # attitude of vtol as a rotation matrix R from body to inertial
        R_bi = state.rot
        R_rotor1 = euler_to_rotation(phi=0, theta=-np.pi/2+state.rotors.item(0), psi=0)
        R_rotor2 = euler_to_rotation(phi=0, theta=-np.pi / 2 + state.rotors.item(1), psi=0)

        # body
        self.vtol_body = self.update_object(
            self.vtol_body,
            self.body_points,
            self.body_index,
            self.body_meshColors,
            R_bi,
            vtol_position)
        # arms
        self.arm1 = self.update_object(
            self.arm1,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm1,
            vtol_position + R_bi @ self.arm1_position)
        self.arm2 = self.update_object(
            self.arm2,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm2,
            vtol_position + R_bi @ self.arm2_position)
        self.arm3 = self.update_object(
            self.arm3,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm3,
            vtol_position + R_bi @ self.arm3_position)
        self.arm4 = self.update_object(
            self.arm4,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm4,
            vtol_position + R_bi @ self.arm4_position)
        # motors
        self.motor1 = self.update_object(
            self.motor1,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor1 @ R_rotor1,
            vtol_position + R_bi @ self.motor1_position)
        self.motor2 = self.update_object(
            self.motor2,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor2 @ R_rotor2,
            vtol_position + R_bi @ self.motor2_position)
        self.motor3 = self.update_object(
            self.motor3,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor3,
            vtol_position + R_bi @ self.motor3_position)
        self.motor4 = self.update_object(
            self.motor4,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor4,
            vtol_position + R_bi @ self.motor4_position)
        # rotors
        self.rotor1 = self.update_object(
            self.rotor1,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor1 @ R_rotor1,
            vtol_position + R_bi @ self.rotor1_position)
        self.rotor2 = self.update_object(
            self.rotor2,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor2 @ R_rotor2,
            vtol_position + R_bi @ self.rotor2_position)
        self.rotor3 = self.update_object(
            self.rotor3,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor3,
            vtol_position + R_bi @ self.rotor3_position)
        self.rotor4 = self.update_object(
            self.rotor4,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor4,
            vtol_position + R_bi @ self.rotor4_position)

    def add_object(self, points, index, colors, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points, index)
        object = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=colors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        return object

    def update_object(self, object, points, index, colors, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points, index)
        object.setMeshData(vertexes=mesh, vertexColors=colors)
        return object

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def points_to_mesh(self, points, index):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[index[0,0]],points[index[0,1]],points[index[0,2]]]])
        for i in range(1, index.shape[0]):
            tmp = np.array([[points[index[i,0]], points[index[i,1]], points[index[i,2]]]])
            mesh = np.concatenate((mesh, tmp), axis=0)
        return mesh

    def get_body_points(self):
        """"
            Points that define the vtol body, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define vtol body parameters


        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        wing_x = self.wing_position.item(0)
        wing_y = self.wing_position.item(1)
        wing_z = self.wing_position.item(2)
        htail_x = self.htail_position.item(0)
        htail_y = self.htail_position.item(1)
        htail_z = self.htail_position.item(2)
        vtail_x = self.vtail_position.item(0)
        vtail_y = self.vtail_position.item(1)
        vtail_z = self.vtail_position.item(2)
        points = np.array([
            [self.body_forward_x, self.body_y1, self.body_z1],  # 0
            [self.body_forward_x, self.body_y1, self.body_z2],  # 1
            [self.body_forward_x/2, self.body_y2, -self.body_z2],  # 2
            [self.body_forward_x/2, self.body_y1, self.body_z2],  # 3
            [self.body_forward_x/2, self.body_y1, self.body_z3],  # 4
            [- self.body_backward_x, self.body_y3, -self.body_z2],  # 5
            [- self.body_backward_x, self.body_y3, self.body_z4],  # 6
            [- self.body_backward_x, -self.body_y3, -self.body_z2],  # 7
            [- self.body_backward_x, -self.body_y3, self.body_z4],  # 8
            [self.body_forward_x/2, -self.body_y2, -self.body_z2],  # 9
            [self.body_forward_x/2, -self.body_y1, self.body_z2],  # 10
            [self.body_forward_x/2, -self.body_y1, self.body_z3],  # 11
            [self.body_forward_x, -self.body_y1, self.body_z1],  # 12
            [self.body_forward_x, -self.body_y1, self.body_z2],  # 13
            [wing_x-self.wing_chord, wing_y+self.wing_span/2, wing_z], #14 wing #1
            [wing_x-self.wing_depth/2, wing_y+self.wing_span/2, wing_z-self.wing_depth/2], #15 wing #2
            [wing_x, wing_y+self.wing_span/2, wing_z], #16 wing #3
            [wing_x-self.wing_depth/2, wing_y+self.wing_span/2, wing_z+self.wing_depth/2], #17 wing #4
            [wing_x-self.wing_chord, wing_y-self.wing_span/2, wing_z], #18 wing #5
            [wing_x-self.wing_depth/2, wing_y-self.wing_span/2, wing_z-self.wing_depth/2], #19 wing #6
            [wing_x, wing_y-self.wing_span/2, wing_z], #20 wing #7
            [wing_x-self.wing_depth/2, wing_y-self.wing_span/2, wing_z+self.wing_depth/2], #21 wing #8
            [htail_x - self.htail_chord, htail_y + self.htail_span/2, htail_z],  # 22 htail #1
            [htail_x - self.htail_depth/2, htail_y + self.htail_span/2, htail_z - self.htail_depth/2],  # 23 htail #2
            [htail_x, htail_y + self.htail_span/2, htail_z],  # 24 wing #3
            [htail_x - self.htail_depth/2, htail_y + self.htail_span/2, htail_z + self.htail_depth/2],  # 25 htail #4
            [htail_x - self.htail_chord, htail_y - self.htail_span/2, htail_z],  # 26 htail #5
            [htail_x - self.htail_depth/2, htail_y - self.htail_span/2, htail_z - self.htail_depth/2],  # 27 htail #6
            [htail_x, htail_y - self.htail_span/2, htail_z],  # 28 wing #7
            [htail_x - self.htail_depth/2, htail_y - self.htail_span/2, htail_z + self.htail_depth/2],  # 29 htail #8
            [vtail_x-self.vtail_chord, vtail_y, vtail_z-self.vtail_span],  # 30 vtail #1
            [vtail_x-self.vtail_chord/2-self.vtail_depth/2, vtail_y-self.vtail_depth/2, vtail_z-self.vtail_span], # 31 vtail #2
            [vtail_x-self.vtail_chord/2, vtail_y, vtail_z-self.vtail_span],  # 32 vtail #3
            [vtail_x-self.vtail_chord/2 - self.vtail_depth/2, self.vtail_depth/2, vtail_z-self.vtail_span], # 33 vtail #4
            [vtail_x-self.vtail_chord, vtail_y, vtail_z],  # 34 vtail #5
            [vtail_x-self.vtail_depth/2, vtail_y-self.vtail_depth/2, vtail_z], # 36 vtail #6
            [vtail_x, vtail_y, vtail_z],  # 36 vtail #7
            [vtail_x-self.vtail_depth/2, vtail_y+self.vtail_depth/2, vtail_z], # 37 vtail #8
        ]).T

        # point index that defines the mesh
        index = np.array([
            [0, 2, 9],  # top
            [0, 12, 9],  # top
            [2, 5, 7], # top
            [2, 9, 7],  # top
            [0, 1, 3],  # right side
            [0, 2, 3],  # right side
            [2, 4, 6], # right side
            [2, 5, 6],  # right side
            [12, 13, 10], # left side
            [12, 9, 10],  # left side
            [9, 7, 8],  # left side
            [9, 11, 8],  # left side
            [7, 8, 6],  # back
            [7, 5, 6],  # back
            [0, 1, 13],  # front top
            [0, 12, 13],  # front top
            [3, 4, 11],  # front-middle
            [3, 10, 11],  # front - middle
            [1, 3, 10],  # bottom
            [1, 13, 10],  # bottom
            [4, 6, 8],  # bottom
            [4, 11, 8],  # bottom
            [14, 15, 18], # wing top
            [15, 19, 18],  # wing top
            [14, 17, 21], # wing bottom
            [14, 18, 21],  # wing bottom
            [15, 16, 20],  # wing leading edge top
            [15, 19, 20],  # wing leading edge top
            [17, 16, 20],  # wing leading edge bottom
            [17, 20, 21],  # wing leading edge bottom
            [14, 15, 16],  # wing right side
            [14, 16, 17],  # wing right side
            [18, 19, 20],  # wing left side
            [18, 20, 21],  # wing left side
            [22, 23, 26],  # htail top
            [23, 27, 26],  # htail top
            [22, 25, 29],  # htail bottom
            [22, 26, 29],  # htail bottom
            [23, 24, 28],  # htail leading edge top
            [23, 27, 28],  # htail leading edge top
            [25, 24, 28],  # htail leading edge bottom
            [25, 28, 29],  # htail leading edge bottom
            [22, 23, 24],  # htail right side
            [22, 24, 25],  # htail right side
            [26, 27, 28],  # htail left side
            [26, 28, 29],  # htail left side
            [30, 31, 34],  # vtail top
            [31, 35, 34],  # vtail top
            [30, 33, 37],  # vtail bottom
            [30, 34, 37],  # vtail bottom
            [31, 32, 36],  # vtail leading edge top
            [31, 35, 36],  # vtail leading edge top
            [33, 32, 36],  # vtail leading edge bottom
            [33, 36, 37],  # vtail leading edge bottom
            [30, 31, 32],  # vtail right side
            [30, 32, 33],  # vtail right side
            [34, 35, 36],  # vtail left side
            [34, 36, 37],  # vtail left side

        ])

        #   define the colors for each face of triangular mesh
        meshColors = np.empty((58, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey1  # top
        meshColors[1] = self.mygrey1  # top
        meshColors[2] = self.mygrey1  # top
        meshColors[3] = self.mygrey1  # top
        meshColors[4] = self.mygrey1  # right side
        meshColors[5] = self.mygrey1  # right side
        meshColors[6] = self.mygrey1  # right side
        meshColors[7] = self.mygrey1  # right side
        meshColors[8] = self.mygrey1  # left side
        meshColors[9] = self.mygrey1  # left side
        meshColors[10] = self.mygrey1  # left side
        meshColors[11] = self.mygrey1  # left side
        meshColors[12] = self.mygrey3  # back
        meshColors[13] = self.mygrey3  # back
        meshColors[14] = self.mygrey2  # front top
        meshColors[15] = self.mygrey2  # front top
        meshColors[16] = self.mygrey2  # front middle
        meshColors[17] = self.mygrey2  # front middle
        meshColors[18] = self.mygrey4  # bottom
        meshColors[19] = self.mygrey4  # bottom
        meshColors[20] = self.mygrey4  # bottom
        meshColors[21] = self.mygrey4  # bottom
        meshColors[22] = self.mygrey1  # wing top
        meshColors[23] = self.mygrey1  # wing top
        meshColors[24] = self.mygrey4  # wing bottom
        meshColors[25] = self.mygrey4  # wing bottom
        meshColors[26] = self.mygrey1  # wing leading edge top
        meshColors[27] = self.mygrey1  # wing leading edge top
        meshColors[28] = self.mygrey4  # wing leading edge bottom
        meshColors[29] = self.mygrey4  # wing leading edge bottom
        meshColors[30] = self.mygrey1  # wing right side
        meshColors[31] = self.mygrey1  # wing right side
        meshColors[32] = self.mygrey1  # wing left side
        meshColors[33] = self.mygrey1  # wing left side
        meshColors[34] = self.mygrey1  # htail top
        meshColors[35] = self.mygrey1  # htail top
        meshColors[36] = self.mygrey4  # htail bottom
        meshColors[37] = self.mygrey4  # htail bottom
        meshColors[38] = self.mygrey1  # htail leading edge top
        meshColors[39] = self.mygrey1  # htail leading edge top
        meshColors[40] = self.mygrey4  # htail leading edge bottom
        meshColors[41] = self.mygrey4  # htail leading edge bottom
        meshColors[42] = self.mygrey1  # htail right side
        meshColors[43] = self.mygrey1  # htail right side
        meshColors[44] = self.mygrey1  # htail left side
        meshColors[45] = self.mygrey1  # htail left side
        meshColors[46] = self.mygrey1  # vtail top
        meshColors[47] = self.mygrey1  # vtail top
        meshColors[48] = self.mygrey4  # vtail bottom
        meshColors[49] = self.mygrey4  # vtail bottom
        meshColors[50] = self.mygrey1  # vtail leading edge top
        meshColors[51] = self.mygrey1  # vtail leading edge top
        meshColors[52] = self.mygrey4  # vtail leading edge bottom
        meshColors[53] = self.mygrey4  # vtail leading edge bottom
        meshColors[54] = self.mygrey1  # vtail right side
        meshColors[55] = self.mygrey1  # vtail right side
        meshColors[56] = self.mygrey1  # vtail left side
        meshColors[57] = self.mygrey1  # vtail left side

        return points, index, meshColors

    def get_arm_points(self):
        # define vtol arm

        # points are in NED coordinates
        points = np.array([
            [0, self.arm_w/2, -self.arm_w/2],  # 0
            [0, self.arm_w/2, self.arm_w/2],  # 1
            [self.arm_l - self.arm_w/2, self.arm_w/2, -self.arm_w/2],  # 2
            [self.arm_l - self.arm_w/2, self.arm_w/2, 0],  # 3
            [self.arm_l - self.arm_w/2, self.arm_w/2, self.arm_w/2],  # 4
            [self.arm_l, 0.75 * self.arm_w/2, 0],  # 5
            [self.arm_l, 0.75 * self.arm_w/2, self.arm_w/2],  # 6
            [0, -self.arm_w/2, -self.arm_w/2],  # 7
            [0, -self.arm_w/2, self.arm_w/2],  # 8
            [self.arm_l - self.arm_w/2, -self.arm_w/2, -self.arm_w/2],  # 9
            [self.arm_l - self.arm_w/2, -self.arm_w/2, 0],  # 10
            [self.arm_l - self.arm_w/2, -self.arm_w/2, self.arm_w/2],  # 11
            [self.arm_l, -0.75 * self.arm_w/2, 0],  # 12
            [self.arm_l, -0.75 * self.arm_w/2, self.arm_w/2],  # 13
            ]).T

        # point index that defines the mesh
        index = np.array([
            [0, 2, 9],  # top
            [0, 7, 9],  # top
            [3, 5, 12],  # top
            [3, 10, 12],  # top
            [0, 2, 4],  # right side
            [0, 1, 4],  # right side
            [3, 4, 6],  # right side
            [3, 5, 6],  # right side
            [7, 8, 11],  # left side
            [7, 9, 11],  # left side
            [10, 11, 13],  # left side
            [10, 12, 13],  # left side
            [1, 8, 11],  # bottom
            [1, 4, 11],  # bottom
            [11, 4, 6],  # bottom
            [11, 13, 6],  # bottom
            [12, 13, 6],  # front
            [12, 5, 6],  # front
            [9, 10, 3],  # front
            [9, 2, 3],  # front
            [0, 1, 8],  # back
            [0, 7, 8],  # back
        ])

        #   define the colors for each face of triangular mesh
        meshColors = np.empty((22, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey1  # top
        meshColors[1] = self.mygrey1  # top
        meshColors[2] = self.mygrey1  # top
        meshColors[3] = self.mygrey1  # top
        meshColors[4] = self.mygrey1  # right side
        meshColors[5] = self.mygrey1  # right side
        meshColors[6] = self.mygrey1  # right side
        meshColors[7] = self.mygrey1  # right side
        meshColors[8] = self.mygrey1  # left side
        meshColors[9] = self.mygrey1  # left side
        meshColors[10] = self.mygrey1  # left side
        meshColors[11] = self.mygrey1  # left side
        meshColors[12] = self.mygrey4  # bottom
        meshColors[13] = self.mygrey4  # bottom
        meshColors[14] = self.mygrey4  # bottom
        meshColors[15] = self.mygrey4  # bottom
        meshColors[16] = self.mygrey2  # front
        meshColors[17] = self.mygrey2  # front
        meshColors[18] = self.mygrey2  # front
        meshColors[19] = self.mygrey2  # front
        meshColors[20] = self.mygrey3  # back
        meshColors[21] = self.mygrey3  # back
        return points, index, meshColors

    def get_motor_points(self):
        # define motor mesh
        height = 0.5 * self.arm_w
        width = 0.5 * self.arm_w
        length = 1 * self.arm_w

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

        # point index that defines the mesh
        index = np.array([
            [0, 1, 2],  # end
            [0, 3, 2],  # end
            [1, 5, 6],  # top
            [1, 2, 6],  # top
            [0, 1, 5],  # right
            [0, 4, 5],  # right
            [0, 4, 7],  # bottom
            [0, 3, 7],  # bottom
            [3, 7, 6],  # left
            [3, 2, 6],  # left
            [4, 5, 6],  # end
            [4, 7, 6],  # end
            ])

        #   define the colors for each face of triangular mesh
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey3  # end
        meshColors[1] = self.mygrey3  # end
        meshColors[2] = self.mygrey3 # top
        meshColors[3] = self.mygrey3  # top
        meshColors[4] = self.mygrey3  # right
        meshColors[5] = self.mygrey3  # right
        meshColors[6] = self.mygrey3  # bottom
        meshColors[7] = self.mygrey3  # bottom
        meshColors[8] = self.mygrey3  # left
        meshColors[9] = self.mygrey3  # left
        meshColors[10] = self.mygrey3  # end
        meshColors[11] = self.mygrey3  # end
        return points, index, meshColors

    def get_rotor_points(self):
        radius = 5 * self.unit_length
        N = 10
        points = np.array([[0, 0, 0]])
        theta = 0
        while theta <= 2*np.pi:
            theta += 2 * np.pi / N
            new_point = np.array([[radius*np.cos(theta), radius*np.sin(theta), 0]])
            points = np.concatenate((points, new_point), axis=0)
        index = np.array([[0, 1, 2]])
        meshColors = np.empty((points.shape[0]-1, 3, 4))
        for i in range(1, (points.shape[0]-1)):
            new_mesh = np.array([[0, i, i+1]])
            index = np.concatenate((index, new_mesh), axis=0)
            meshColors[i] = self.mygrey4
        return points.T, index, meshColors

    def get_rotor_points_old(self):
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



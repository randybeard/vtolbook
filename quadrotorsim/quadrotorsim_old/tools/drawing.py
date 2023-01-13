"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""

import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation
from chap11.dubins_parameters import dubinsParameters

class DrawQuadrotor():
    def __init__(self, state, window):
        """
        Draw the quadrotor

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.p  # inertial position (NED)
            state.R  # rotation R_b^i
        """
        self.unit_length = 0.5
        quadrotor_position = state.pos  # NED coordinates
        R_bi = state.rot  # body to inertial rotation
        # convert North-East Down to East-North-Up for rendering
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # body
        self.body_points, self.body_index, self.body_meshColors = self.get_body_points()
        self.quadrotor_body = self.add_object(
            self.body_points,
            self.body_index,
            self.body_meshColors,
            R_bi,
            quadrotor_position)
        window.addItem(self.quadrotor_body)  # add body to plot

        # arms
        self.arm_w = 2. * self.unit_length # width of arm
        self.arm_l = 12. * self.unit_length  # length of arm
        arm_front_x = 6. * self.unit_length  # x position of front arm
        arm_front_y = 3. * self.unit_length  # y position of front arm
        arm_front_z = 0.8 * self.unit_length  # z position of front arm
        arm_back_x = -1. * self.unit_length  # x position of back arm
        arm_back_y = 2. * self.unit_length  # y position of back arm
        arm_back_z = 0 * self.unit_length  # z position of back arm
        self.arm_points, self.arm_index, self.arm_meshColors = self.get_arm_points()
        # arm 1
        self.R_arm1 = euler_to_rotation(0.0, 0.0, np.pi/4)
        self.arm1_position = np.array([[arm_front_x, arm_front_y, arm_front_z]]).T
        self.arm1 = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm1,
            quadrotor_position + R_bi @ self.arm1_position)
        window.addItem(self.arm1)
        # arm 2
        self.R_arm2 = euler_to_rotation(0.0, 0.0, -np.pi/4)
        self.arm2_position = np.array([[arm_front_x, -arm_front_y, arm_front_z]]).T
        self.arm2 = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm2,
            quadrotor_position + R_bi @ self.arm2_position)
        window.addItem(self.arm2)
        # arm 3
        self.R_arm3 = euler_to_rotation(0.0, 0.0, 3*np.pi/4)
        self.arm3_position = np.array([[arm_back_x, arm_back_y, arm_back_z]]).T
        self.arm3 = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm3,
            quadrotor_position + R_bi @ self.arm3_position)
        window.addItem(self.arm3)
        # arm 4
        self.R_arm4 = euler_to_rotation(0.0, 0.0, -3*np.pi/4)
        self.arm4_position = np.array([[arm_back_x, -arm_back_y, arm_back_z]]).T
        self.arm4 = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm4,
            quadrotor_position + R_bi @ self.arm4_position)
        window.addItem(self.arm4)

        # motors
        self.motor_points, self.motor_index, self.motor_meshColors = self.get_motor_points()
        self.R_motor1 = euler_to_rotation(0.0, np.pi/2, np.pi/4)
        self.motor1_position = self.arm1_position + self.R_arm1 @ np.array([[self.arm_l, 0, 0]]).T
        self.motor1 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor1,
            quadrotor_position + R_bi @ self.motor1_position)
        window.addItem(self.motor1)
        self.R_motor2 = euler_to_rotation(0.0, np.pi/2, -np.pi/4)
        self.motor2_position = self.arm2_position + self.R_arm2 @ np.array([[self.arm_l, 0, 0]]).T
        self.motor2 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor2,
            quadrotor_position + R_bi @ self.motor2_position)
        window.addItem(self.motor2)
        self.R_motor3 = euler_to_rotation(0.0, np.pi/2, 3*np.pi/4)
        self.motor3_position = self.arm3_position + self.R_arm3 @ np.array([[self.arm_l, 0, 0]]).T
        self.motor3 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor3,
            quadrotor_position + R_bi @ self.motor3_position)
        window.addItem(self.motor3)
        self.R_motor4 = euler_to_rotation(0.0, np.pi/2, -3*np.pi/4)
        self.motor4_position = self.arm4_position + self.R_arm4 @ np.array([[self.arm_l, 0, 0]]).T
        self.motor4 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor4,
            quadrotor_position + R_bi @ self.motor4_position)
        window.addItem(self.motor4)

        # rotors
        self.rotor_points, self.rotor_index, self.rotor_meshColors = self.get_rotor_points()
        self.R_rotor1 = euler_to_rotation(0.0, 0.0, 0.0)
        self.rotor1_position = self.arm1_position + self.R_arm1 @ np.array([[self.arm_l, 0, -self.arm_w]]).T
        self.rotor1 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor1,
            quadrotor_position + R_bi @ self.rotor1_position)
        window.addItem(self.rotor1)
        self.R_rotor2 = euler_to_rotation(0.0, 0.0, 0.0)
        self.rotor2_position = self.arm2_position + self.R_arm2 @ np.array([[self.arm_l, 0, -self.arm_w]]).T
        self.rotor2 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor2,
            quadrotor_position + R_bi @ self.rotor2_position)
        window.addItem(self.rotor2)
        self.R_rotor3 = euler_to_rotation(0.0, 0.0, 0.0)
        self.rotor3_position = self.arm3_position + self.R_arm3 @ np.array([[self.arm_l, 0, -self.arm_w]]).T
        self.rotor3 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor3,
            quadrotor_position + R_bi @ self.rotor3_position)
        window.addItem(self.rotor3)
        self.R_rotor4 = euler_to_rotation(0.0, 0.0, 0.0)
        self.rotor4_position = self.arm4_position + self.R_arm4 @ np.array([[self.arm_l, 0, -self.arm_w]]).T
        self.rotor4 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor4,
            quadrotor_position + R_bi @ self.rotor4_position)
        window.addItem(self.rotor4)

    def update(self, state):
        # NED coordinates of quadrotor
        quadrotor_position = state.pos
        # attitude of vtol as a rotation matrix R from body to inertial
        R_bi = state.rot

        # body
        self.quadrotor_body = self.update_object(
            self.quadrotor_body,
            self.body_points,
            self.body_index,
            self.body_meshColors,
            R_bi,
            quadrotor_position)
        # arms
        self.arm1 = self.update_object(
            self.arm1,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm1,
            quadrotor_position + R_bi @ self.arm1_position)
        self.arm2 = self.update_object(
            self.arm2,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm2,
            quadrotor_position + R_bi @ self.arm2_position)
        self.arm3 = self.update_object(
            self.arm3,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm3,
            quadrotor_position + R_bi @ self.arm3_position)
        self.arm4 = self.update_object(
            self.arm4,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            R_bi @ self.R_arm4,
            quadrotor_position + R_bi @ self.arm4_position)
        # motors
        self.motor1 = self.update_object(
            self.motor1,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor1,
            quadrotor_position + R_bi @ self.motor1_position)
        self.motor2 = self.update_object(
            self.motor2,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor2,
            quadrotor_position + R_bi @ self.motor2_position)
        self.motor3 = self.update_object(
            self.motor3,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor3,
            quadrotor_position + R_bi @ self.motor3_position)
        self.motor4 = self.update_object(
            self.motor4,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ self.R_motor4,
            quadrotor_position + R_bi @ self.motor4_position)
        # rotors
        self.rotor1 = self.update_object(
            self.rotor1,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor1,
            quadrotor_position + R_bi @ self.rotor1_position)
        self.rotor2 = self.update_object(
            self.rotor2,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor2,
            quadrotor_position + R_bi @ self.rotor2_position)
        self.rotor3 = self.update_object(
            self.rotor3,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor3,
            quadrotor_position + R_bi @ self.rotor3_position)
        self.rotor4 = self.update_object(
            self.rotor4,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ self.R_rotor4,
            quadrotor_position + R_bi @ self.rotor4_position)

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
            Points that define the quadrotor body, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define Quadrotor body parameters

        body_forward_x = 10 * self.unit_length
        body_backward_x = 7 * self.unit_length
        body_y1 = 4 * self.unit_length
        body_y2 = 5 * self.unit_length
        body_y3 = 2 * self.unit_length
        body_z1 = 0 * self.unit_length
        body_z2 = 2 * self.unit_length
        body_z3 = 4 * self.unit_length
        body_z4 = 1 * self.unit_length

        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        points = np.array([
            [body_forward_x, body_y1, body_z1],  # 0
            [body_forward_x, body_y1, body_z2],  # 1
            [body_forward_x / 2, body_y2, -body_z2],  # 2
            [body_forward_x / 2, body_y1, body_z2],  # 3
            [body_forward_x / 2, body_y1, body_z3],  # 4
            [- body_backward_x, body_y3, -body_z2],  # 5
            [- body_backward_x, body_y3, body_z4],  # 6
            [- body_backward_x, -body_y3, -body_z2],  # 7
            [- body_backward_x, -body_y3, body_z4],  # 8
            [body_forward_x / 2, -body_y2, -body_z2],  # 9
            [body_forward_x / 2, -body_y1, body_z2],  # 10
            [body_forward_x / 2, -body_y1, body_z3],  # 11
            [body_forward_x, -body_y1, body_z1],  # 12
            [body_forward_x, -body_y1, body_z2],  # 13
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
            ])

        #   define the colors for each face of triangular mesh
        # #red = np.array([1., 0., 0., 1])
        # red = np.array([211, 68, 63, 256])/256
        # #green = np.array([0., 1., 0., 1])
        # green = np.array([63, 211, 105, 256])/256.
        # blue = np.array([0., 0., 1., 1])
        # yellow = np.array([1., 1., 0., 1])
        mygrey1 = np.array([0.8, 0.8, 0.8, 1])  # light
        mygrey2 = np.array([0.6, 0.6, 0.6, 1])
        mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        mygrey4 = np.array([0.3, 0.3, 0.3, 1])  # dark
        meshColors = np.empty((22, 3, 4), dtype=np.float32)
        meshColors[0] = mygrey1  # top
        meshColors[1] = mygrey1  # top
        meshColors[2] = mygrey1  # top
        meshColors[3] = mygrey1  # top
        meshColors[4] = mygrey1  # right side
        meshColors[5] = mygrey1  # right side
        meshColors[6] = mygrey1  # right side
        meshColors[7] = mygrey1  # right side
        meshColors[8] = mygrey1  # left side
        meshColors[9] = mygrey1  # left side
        meshColors[10] = mygrey1  # left side
        meshColors[11] = mygrey1  # left side
        meshColors[12] = mygrey3  # back
        meshColors[13] = mygrey3  # back
        meshColors[14] = mygrey2  # front top
        meshColors[15] = mygrey2  # front top
        meshColors[16] = mygrey2  # front middle
        meshColors[17] = mygrey2  # front middle
        meshColors[18] = mygrey4  # bottom
        meshColors[19] = mygrey4  # bottom
        meshColors[20] = mygrey4  # bottom
        meshColors[21] = mygrey4  # bottom
        return points, index, meshColors

    def get_arm_points(self):
        # define Quadrotor arm

        # points are in NED coordinates
        points = np.array([
            [0, self.arm_w / 2, -self.arm_w / 2],  # 0
            [0, self.arm_w / 2, self.arm_w / 2],  # 1
            [self.arm_l - self.arm_w / 2, self.arm_w / 2, -self.arm_w / 2],  # 2
            [self.arm_l - self.arm_w / 2, self.arm_w / 2, 0],  # 3
            [self.arm_l - self.arm_w / 2, self.arm_w / 2, self.arm_w / 2],  # 4
            [self.arm_l, 0.75 * self.arm_w / 2, 0],  # 5
            [self.arm_l, 0.75 * self.arm_w / 2, self.arm_w / 2],  # 6
            [0, -self.arm_w / 2, -self.arm_w / 2],  # 7
            [0, -self.arm_w / 2, self.arm_w / 2],  # 8
            [self.arm_l - self.arm_w / 2, -self.arm_w / 2, -self.arm_w / 2],  # 9
            [self.arm_l - self.arm_w / 2, -self.arm_w / 2, 0],  # 10
            [self.arm_l - self.arm_w / 2, -self.arm_w / 2, self.arm_w / 2],  # 11
            [self.arm_l, -0.75 * self.arm_w / 2, 0],  # 12
            [self.arm_l, -0.75 * self.arm_w / 2, self.arm_w / 2],  # 13
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
        # #red = np.array([1., 0., 0., 1])
        # red = np.array([211, 68, 63, 256])/256
        # #green = np.array([0., 1., 0., 1])
        # green = np.array([63, 211, 105, 256])/256.
        # blue = np.array([0., 0., 1., 1])
        # yellow = np.array([1., 1., 0., 1])
        mygrey1 = np.array([0.8, 0.8, 0.8, 1])  # light
        mygrey2 = np.array([0.6, 0.6, 0.6, 1])
        mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        mygrey4 = np.array([0.3, 0.3, 0.3, 1])  # dark
        meshColors = np.empty((22, 3, 4), dtype=np.float32)
        meshColors[0] = mygrey1  # top
        meshColors[1] = mygrey1  # top
        meshColors[2] = mygrey1  # top
        meshColors[3] = mygrey1  # top
        meshColors[4] = mygrey1  # right side
        meshColors[5] = mygrey1  # right side
        meshColors[6] = mygrey1  # right side
        meshColors[7] = mygrey1  # right side
        meshColors[8] = mygrey1  # left side
        meshColors[9] = mygrey1  # left side
        meshColors[10] = mygrey1  # left side
        meshColors[11] = mygrey1  # left side
        meshColors[12] = mygrey4  # bottom
        meshColors[13] = mygrey4  # bottom
        meshColors[14] = mygrey4  # bottom
        meshColors[15] = mygrey4  # bottom
        meshColors[16] = mygrey2  # front
        meshColors[17] = mygrey2  # front
        meshColors[18] = mygrey2  # front
        meshColors[19] = mygrey2  # front
        meshColors[20] = mygrey3  # back
        meshColors[21] = mygrey3  # back
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
        # #red = np.array([1., 0., 0., 1])
        # red = np.array([211, 68, 63, 256])/256
        # green = np.array([0., 1., 0., 1])
        # blue = np.array([66, 161, 244, 256])/256.
        # yellow = np.array([1., 1., 0., 1])
        mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = mygrey3  # end
        meshColors[1] = mygrey3  # end
        meshColors[2] = mygrey3 # top
        meshColors[3] = mygrey3  # top
        meshColors[4] = mygrey3  # right
        meshColors[5] = mygrey3  # right
        meshColors[6] = mygrey3  # bottom
        meshColors[7] = mygrey3  # bottom
        meshColors[8] = mygrey3  # left
        meshColors[9] = mygrey3  # left
        meshColors[10] = mygrey3  # end
        meshColors[11] = mygrey3  # end
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
        mygrey4 = np.array([0.3, 0.3, 0.3, 1])
        index = np.array([[0, 1, 2]])
        meshColors = np.empty((points.shape[0]-1, 3, 4))
        for i in range(1, (points.shape[0]-1)):
            new_mesh = np.array([[0, i, i+1]])
            index = np.concatenate((index, new_mesh), axis=0)
            meshColors[i] = mygrey4
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


class DrawWaypoints():
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

class DrawMap():
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



import sys
sys.path.append('..')
import numpy as np

import cv2
import airsim
from message_types.msg_state import msgState
from tools.rotations import Euler2Quaternion, Euler2Rotation
from airsim.types import Pose

class airsimDemo:
    def __init__(self):
        self.scale = 1.0
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.pn_offset = 0.0
        self.pe_offset = 0.0
        self.pd_offset = -0.3

        self.follower_roll = 0.0
        self.follower_pitch = np.deg2rad(-20)
        self.follower_pose_offset = np.array([-3., 0., -1.])

    @staticmethod
    def parse_camera(in_images):
        fpv_img1d = np.fromstring(in_images[0].image_data_uint8, dtype=np.uint8) # get numpy array
        fpv_image = fpv_img1d.reshape(in_images[0].height, in_images[0].width, 3) # reshape array to 3 channel image array H X W X 3

        follower_img1d = np.fromstring(in_images[1].image_data_uint8, dtype=np.uint8) # get numpy array
        follower_image = follower_img1d.reshape(in_images[1].height, in_images[1].width, 3) # reshape array to 3 channel image array H X W X 3

        # print(img1d.shape, image.shape, end='\r')
        cv2.imshow('FPV Image', fpv_image)
        cv2.imshow('Follower Image', follower_image)
        cv2.waitKey(1)

    def update(self, state_update: msgState):

        follower_pose = self.new_follower_pose(state_update)
        vehicle_pose = self.new_vehicle_pose(state_update)

        self.client.simPause(False)

        self.client.simSetVehiclePose(vehicle_pose,False)
        self.client.simSetCameraPose("VTOL_0_follower", follower_pose)

        self.parse_camera(self.client.simGetImages([
            airsim.ImageRequest("VTOL_0_front_center", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("VTOL_0_follower", airsim.ImageType.Scene, False, False),
            ]))
        collision = self.client.simGetCollisionInfo().has_collided

        self.client.simPause(True)

        if collision:
            print("Vehicle has collided with obstacle at ({:.3f},{:.3f},{:.3f})".format(
                vehicle_pose.position.x_val,vehicle_pose.position.y_val,vehicle_pose.position.z_val))

    def new_vehicle_pose(self, state_update):
        pose = Pose()
        pose.position.x_val = state_update.pn*self.scale + self.pn_offset
        pose.position.y_val = state_update.pe*self.scale + self.pe_offset
        pose.position.z_val = -state_update.h*self.scale + self.pd_offset
        quat = Euler2Quaternion(state_update.phi, state_update.theta, state_update.psi)
        pose.orientation.w_val = quat.item(0)
        pose.orientation.x_val = quat.item(1)
        pose.orientation.y_val = quat.item(2)
        pose.orientation.z_val = quat.item(3)

        return pose

    def new_follower_pose(self, vehicle_state):
        # camera pose relative to vehicle body frame
        follower_pose = Pose()

        # rotation from yawed (level) frame to body frame
        R_l2b = Euler2Rotation(vehicle_state.phi, vehicle_state.theta, 0.).T

        follower_pose_offset_b = R_l2b @ self.follower_pose_offset

        follower_pose.position.x_val = follower_pose_offset_b[0]
        follower_pose.position.y_val = follower_pose_offset_b[1]
        follower_pose.position.z_val = follower_pose_offset_b[2]

        quat = Euler2Quaternion(self.follower_roll - vehicle_state.phi, self.follower_pitch - vehicle_state.theta, 0.)
        follower_pose.orientation.w_val = quat.item(0)
        follower_pose.orientation.x_val = quat.item(1)
        follower_pose.orientation.y_val = quat.item(2)
        follower_pose.orientation.z_val = quat.item(3)

        return follower_pose

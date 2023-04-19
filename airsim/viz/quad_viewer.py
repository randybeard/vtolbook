import airsim
from message_types.msg_state import MsgState
from tools.rotations import rotation_to_quaternion
import numpy as np
import time
import os
import keyboard

class QuadViewer:
    def __init__(self):
        self._client = airsim.MultirotorClient("10.32.114.170")
        self._client.confirmConnection()

    def update(self, state):
        # 'state' is of type MsgState

        poseObj = self._client.simGetVehiclePose()
        poseObj.position.x_val = state.pos.item(0)
        poseObj.position.y_val = state.pos.item(1)
        poseObj.position.z_val = state.pos.item(2)
        # get the quaternion of the state
        quat = rotation_to_quaternion(state.rot).reshape((4))
        poseObj.orientation.w_val = quat.item(0)
        poseObj.orientation.x_val = quat.item(1)
        poseObj.orientation.y_val = quat.item(2)
        poseObj.orientation.z_val = quat.item(3)

        self._client.simSetVehiclePose(pose=poseObj, ignore_collision=True)

    def get_image(self, camera_name="down"):
        im = self._client.simGetImages([airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)])[0]
        
        # get numpy array
        img1d = np.fromstring(im.image_data_uint8, dtype=np.uint8) 

        # reshape array to 3 channel image array H X W X 3
        img_rgb = img1d.reshape(im.height, im.width, 3)

        # original image is fliped vertically
        # img_rgb = np.flipud(img_rgb)

        return img_rgb
    
    def spawn_target(self):
        # print(os.getcwd())
        # # spawn red sphere in front of quadrotor
        self.target_name = "my_green_object"

        filename = "green.jpg"

        for root,dir,files in os.walk(os.getcwd()):
            if filename in files:
                path = os.path.join(root,filename)
        
        if path is None:
            for root,dir,files in os.walk(os.getcwd() + ".."):
                "here"
                if filename in files:
                    path = os.path.join(root,filename)

        pose = self._client.simGetVehiclePose()
        pose.position.z_val = pose.position.z_val
        pose.position.x_val = pose.position.x_val + 10
        pose.position.y_val = pose.position.y_val
        scale = airsim.Vector3r(1, 1, 1)
        # spawn a cube a meter below the vehicle
        if np.isnan(self._client.simGetObjectPose(self.target_name).position.x_val):
            mycube = self._client.simSpawnObject(self.target_name, 'sphere', pose, scale, physics_enabled=True)
        else:
            self._client.simDestroyObject(self.target_name)
            time.sleep(.1)
            self._client.simSpawnObject(self.target_name, 'sphere', pose, scale, physics_enabled=True)


        self._client.simSetObjectMaterialFromTexture(self.target_name, path)
        time.sleep(2)

    def update_target(self,velocity,dt):
        velocity = np.array(velocity)           

        pose = self._client.simGetObjectPose(self.target_name)
        pose.position.x_val = pose.position.x_val + velocity.item(0)*dt
        pose.position.y_val = pose.position.y_val + velocity.item(1)*dt
        pose.position.z_val = pose.position.z_val + velocity.item(2)*dt
        
        self._client.simSetObjectPose(self.target_name, pose, True)

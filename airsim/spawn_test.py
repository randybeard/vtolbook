import airsim
import time
import numpy as np
import cv2

client = airsim.MultirotorClient()
client.confirmConnection()

pose = client.simGetVehiclePose()
pose.position.z_val = pose.position.z_val - 2
pose.position.x_val = pose.position.x_val + 2
scale = airsim.Vector3r(1, 1, 1)
# spawn a cube a meter below the vehicle
mycube = client.simSpawnObject('my_cube', 'sphere', pose, scale, physics_enabled=True)
time.sleep(1)
client.simSetObjectMaterialFromTexture("my_cube","/home/aaron/Downloads/red.jpg")
time.sleep(0.1)
# move the cube +10 meters horizontally


# dt = .01
# vector = np.array([1,0,0])
# while True:
#     pose = client.simGetObjectPose('my_cube')
#     pose.position.x_val = pose.position.x_val + vector.item(0)*dt
#     client.simSetObjectPose('my_cube', pose, True)

#     im = client.simGetImages([airsim.ImageRequest("forward", airsim.ImageType.Scene, False, False)])[0]

#     img1d = np.fromstring(im.image_data_uint8, dtype=np.uint8) 

#     # reshape array to 3 channel image array H X W X 3
#     img_rgb = img1d.reshape(im.height, im.width, 3)

#     cv2.imshow("forward",img_rgb)
#     cv2.waitKey(1)


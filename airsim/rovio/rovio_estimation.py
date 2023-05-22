import time
import rospy
import numpy as np
import pathlib
import sys

from copy import deepcopy

# this line gets the parent folder of the parent folder of the current file
# i.e. vtolbook/airsim/. If your sim file is in the airsim folder, you don't need this
directory = pathlib.Path(__file__).parent.parent
# this is so we can have the base folder of the project (vtolbook/airsim) in the path
sys.path.append(str(directory))

import parameters.quadrotor_parameters as QUAD
import parameters.simulation_parameters as SIM
from viz.quad_viewer import QuadViewer
from dynamics.quad_dynamics import QuadDynamics
from controllers.jakes_controller import Autopilot
from trajectory_generators.circular_trajectory import TrajectoryGenerator
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from tools.rotations import quaternion_to_rotation, rotation_to_euler

from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import cv2

class AirsimNode:
    airsim_viz = QuadViewer()
    quadrotor = QuadDynamics(SIM.ts_simulation, QUAD)
    autopilot = Autopilot(SIM.ts_simulation, QUAD)
    traj_gen = TrajectoryGenerator(SIM.ts_simulation)
    delta = MsgDelta()
    sim_time = SIM.start_time
    next_frame = SIM.start_time
    body_to_rovio = np.array([[0., 0., -1.],[0., 1., 0.],[1., 0., 0.]])
    odom_to_inertial = np.array([[0.,0.,-1.],[0.,1.,0.],[1.,0.,0.]]).T

    initiated = False

    def __init__(self):
        self.imu_pub = rospy.Publisher("imu0", Imu, queue_size=1)
        self.image_pub = rospy.Publisher("cam0/image_raw", Image, queue_size=1)
        rospy.Subscriber("rovio/odometry", Odometry, self.odometry_callback, queue_size=1)

    def publish_data(self):
        # get the desired trajectory from the trajectory generator
        trajectory = self.traj_gen.update()

        # get the estimated state (the estimate is ground truth here)
        if not self.initiated:
            estimated_state = deepcopy(self.quadrotor.true_state) # TODO: replace this with the estimated states
        else:
            estimated_state = deepcopy(self.estimated_state)

        # get the commands from the controller
        delta, commanded_states = self.autopilot.update(trajectory, estimated_state)

        # update the dynamics and simulation
        self.quadrotor.update(delta)

        # get the IMU data
        omega = self.quadrotor.true_state.omega
        acc = self.quadrotor.acceleration

        # rotate the omega and acc into the ROVIO expected frame (FLU)
        omega = self.body_to_rovio @ omega
        acc = self.body_to_rovio @ acc
        # add noise?

        # publish the IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.angular_velocity.x = omega[0]
        imu_msg.angular_velocity.y = omega[1]
        imu_msg.angular_velocity.z = omega[2]
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]

        self.imu_pub.publish(imu_msg)

        # update AirSim
        self.airsim_viz.update(self.quadrotor.true_state)

        # check if we should publish a camera frame
        if self.sim_time >= self.next_frame:
            # get the frame
            frame = self.airsim_viz.get_image("forward")

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            bridge = CvBridge()
            img_msg = bridge.cv2_to_imgmsg(frame_gray, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(img_msg)
            self.next_frame += SIM.ts_video

        # increment the simulation
        self.sim_time += SIM.ts_simulation
    def odometry_callback(self, odom:Odometry):
        if self.sim_time > 5:
            self.initiated = True
        self.estimated_state = MsgState()

        quat = odom.pose.pose.orientation
        quaternion = np.array([[quat.w],[quat.x],[quat.y],[quat.z]])
        self.estimated_state.rot = np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0]])@quaternion_to_rotation(quaternion)

        position = odom.pose.pose.position
        self.estimated_state.pos[0] = position.x
        self.estimated_state.pos[1] = position.y
        self.estimated_state.pos[2] = position.z
        self.estimated_state.pos = self.quadrotor.true_state.pos#self.odom_to_inertial @ self.estimated_state.pos
        
        velocity = odom.twist.twist.linear
        self.estimated_state.vel[0] = velocity.x
        self.estimated_state.vel[1] = velocity.y
        self.estimated_state.vel[2] = velocity.z
        self.estimated_state.vel = self.quadrotor.true_state.vel#self.estimated_state.rot @ self.body_to_rovio.T @ self.estimated_state.vel

        omega = odom.twist.twist.angular
        self.estimated_state.omega[0] = omega.x
        self.estimated_state.omega[1] = omega.y
        self.estimated_state.omega[2] = omega.z
        self.estimated_state.omega = self.quadrotor.true_state.omega#self.body_to_rovio.T @ self.estimated_state.omega


if __name__ == "__main__":
    rospy.init_node("airsim_node",anonymous=True)
    node = AirsimNode()

    publish_rate = 1./SIM.ts_simulation
    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():
        node.publish_data()
        rate.sleep()

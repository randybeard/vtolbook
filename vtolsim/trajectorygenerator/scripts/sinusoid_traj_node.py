#!/usr/bin/env python3

# system imports
import numpy as np

# ROS imports
import rospy
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command, Status
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# local imports
from linear_trajectory_member import LinearTrajectoryMember
from sinusoidal_trajectory_member import SinusoidalTrajectoryMember
from trajectory import Trajectory


class SinusoidTrajNode:
    """
    ROS node that implements sinusoidal and linear
    trajectory members and publishes them
    """
    def __init__(self):
        self.traj_generator = self.setup_trajectory_generator()
        self.armed = False
        self.start_offset = 0.0  # time offset from when system is armed
        self.kr = rospy.get_param(
            'trajectory_generator/kr', 2)

        # Publishers and Subscribers
        rospy.Subscriber("status", Status, self.statusCallback)
        rospy.Subscriber(
            "odom", Odometry, self.stateCallback, tcp_nodelay=True)
        self.traj_pub = rospy.Publisher("trajectory", Float32MultiArray,
                                        queue_size=1)

        # ROS Message Formatting
        Nr = self.traj_generator.Nr
        self.traj_msg = Float32MultiArray()
        self.traj_msg.layout.dim.append(MultiArrayDimension())
        self.traj_msg.layout.dim.append(MultiArrayDimension())
        self.traj_msg.layout.dim[0].label = "height"
        self.traj_msg.layout.dim[1].label = "width"
        self.traj_msg.layout.dim[0].size = Nr
        self.traj_msg.layout.dim[1].size = self.kr+1
        self.traj_msg.layout.dim[0].stride = Nr*(self.kr+1)
        self.traj_msg.layout.dim[1].stride = 1
        self.traj_msg.layout.data_offset = 0

    def run(self):
        """This function runs until ROS node is killed"""
        rospy.spin()

    def setup_trajectory_generator(self):
        raw_members = rospy.get_param(
            'trajectory_generator/member_definitions')
        members = []
        for member in raw_members:
            if member['type'] == 'sin':
                members.append(SinusoidalTrajectoryMember(*member['data']))
            elif member['type'] == 'lin':
                members.append(LinearTrajectoryMember(*member['data']))
            else:
                print('[TrajectoryNode] Error - undefined member type')

        ts = rospy.get_param(
            'trajectory_generator/time_scale', 1.0)
        trajectory = Trajectory(members, time_scale=ts)

        return trajectory

    def statusCallback(self, msg):
        """
        Callback waits for system to arm before calculating time to
        start trajectory
        """
        if not self.armed and msg.armed:
            self.start_offset = rospy.Time.now().to_sec()
            print("[TrajectoryNode] Trajectory Time Offset Captured as: ",
                  self.start_offset, " seconds")
        self.armed = msg.armed

    def stateCallback(self, msg):
        """
        Callback publishes trajectory at same rate as incoming state messages
        """
        if self.armed:
            self.publish_trajectory()

    def publish_trajectory(self):
        """
        Callback publishes trajectory information with time offset to
        align trajectory start with arm time
        """
        t = rospy.Time.now().to_sec() - self.start_offset
        self.traj_msg.data = \
            self.traj_generator.evalUpToKr(t, self.kr).flatten().tolist()
        self.traj_pub.publish(self.traj_msg)


if __name__ == '__main__':
    rospy.init_node('sinusoid_trajectory_generator', anonymous=True)
    try:
        ros_node = SinusoidTrajNode()
        ros_node.run()
    except rospy.ROSInterruptException:
        pass

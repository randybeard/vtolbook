#!/usr/bin/env python3

# system imports
import numpy as np

# ROS imports
import rospy
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command, Status

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseArray

# local imports
from trajectory import Trajectory
from bspline_trajectory_generator \
    import BSplineTrajectoryGenerator

import utils.math_tools as mt


class BSplineTrajNode:
    """
    Subscribes to waypoints published by A* path planner, implements minimum
    snap spline trajectories through waypoints, and publishes path and
    derivatives for trajectory follower
    """
    def __init__(self):
        self.armed = False
        self.start_offset = 0.0  # time offset from when system is armed

        self.waypoints = []
        self.pos_trajectory = None
        self.yaw_trajectory = None

        self.bspline_generator = BSplineTrajectoryGenerator()

        # true states of the vehicle required for setting the first knot point
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.current_yaw = 0.0
        self.current_yaw_rate = 0.0

        # Publishers and Subscribers
        rospy.Subscriber("status", Status, self.statusCallback)
        rospy.Subscriber(
            "odom",
            Odometry,
            self.stateCallback,
            tcp_nodelay=True)
        rospy.Subscriber("waypoints", PoseArray, self.waypointCallback)

        self.traj_pub = rospy.Publisher("trajectory", Float32MultiArray,
                                        queue_size=5)

        self.degree = rospy.get_param('trajectory_generator/degree')

        # check for waypoints defined in ros params
        self.use_static_trajectory = False
        static_waypoints = rospy.get_param(
            'trajectory_generator/static_waypoints')
        if static_waypoints is not None:
            self.use_static_trajectory = True
            self.create_trajectory(static_waypoints)

        # Format trajectory message
        Nr = 4
        self.traj_msg = Float32MultiArray()
        self.traj_msg.layout.dim.append(MultiArrayDimension())
        self.traj_msg.layout.dim.append(MultiArrayDimension())
        self.traj_msg.layout.dim[0].label = "height"
        self.traj_msg.layout.dim[1].label = "width"
        self.traj_msg.layout.dim[0].size = Nr
        self.traj_msg.layout.dim[1].size = self.degree
        self.traj_msg.layout.dim[0].stride = Nr*(self.degree)
        self.traj_msg.layout.dim[1].stride = 1
        self.traj_msg.layout.data_offset = 0

    @staticmethod
    def run():
        """This function runs until ROS node is killed"""
        rospy.spin()

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
        position = msg.pose.pose.position   # NED position in i frame
        orient = msg.pose.pose.orientation  # orientation in quaternion
        linear = msg.twist.twist.linear     # linear velocity in i? frame
        angular = msg.twist.twist.angular   # angular velocity in b frame

        self.current_position = np.array([position.x, position.y, position.z])
        self.current_velocity = np.array([linear.x, linear.y, linear.z])
        self.current_yaw = mt.quat2euler(
            np.array([orient.w, orient.x, orient.y, orient.z])).item(2)
        self.current_yaw_rate = angular.z

        # only publish trajectory if controller is armed
        if self.armed:
            self.publish_trajectory()

    def waypointCallback(self, msg):
        """
        Get new waypoints, unpack them, and create a new spline trajectory
        """
        # ignore waypoint callback if just flying example waypoints
        if self.use_static_trajectory:
            return

        waypoints_msgs = msg.poses

        waypoints = []

        for pose in waypoints_msgs:
            waypoints.append(
                np.array([pose.position.x, pose.position.y, pose.position.z]))

        self.create_trajectory(waypoints)

    def create_trajectory(self, waypoints):
        """
        Generate a trajectory with waypoints
        """
        waypoints = np.array(waypoints)
        # ensure points are along rows - assumes more than 3 waypoints!
        if waypoints.shape[0] < waypoints.shape[1]:
            waypoints = waypoints.T

        self.pos_trajectory = (self.bspline_generator
                               .generate_trajectory(waypoints, self.degree))
        self.pos_trajectory.time_scale = self.time_scale

        # get velocities at each knot point to calculate desired heading
        vel_list = []
        for i in range(waypoints.shape[1]):
            vel_list.append(self.pos_trajectory.eval(i/self.time_scale, 1))

        yaw_points = mt.velocities_to_headings(vel_list, self.current_yaw)

        self.yaw_trajectory = (self.bspline_generator
                               .generate_trajectory(yaw_points, self.degree))
        self.yaw_trajectory.time_scale = self.time_scale

    def publish_trajectory(self):
        """
        Callback publishes trajectory information with time offset to
        align trajectory start with arm time
        """

        if self.pos_trajectory is None or self.yaw_trajectory is None:
            return

        t = rospy.Time.now().to_sec() - self.start_offset

        # Concatenate position and yaw derivatives
        pos_traj_data = (self.pos_trajectory.evalUpToKr(t, self.kr)
                         .flatten().tolist())
        yaw_traj_data = (self.yaw_trajectory.evalUpToKr(t, self.kr)
                         .flatten().tolist())
        self.traj_msg.data = pos_traj_data + yaw_traj_data

        self.traj_pub.publish(self.traj_msg)


if __name__ == '__main__':
    rospy.init_node('bspline_trajectory_generator', anonymous=True)
    try:
        ros_node = BSplineTrajNode()
        ros_node.run()
    except rospy.ROSInterruptException:
        pass

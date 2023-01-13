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
from polynomial_trajectory_generator \
    import PolynomialTrajectoryGenerator as PTG
from basis.standard_basis_member import StandardBasisMember as SBM

import utils.math_tools as mt


class SplineTrajNode:
    """
    Subscribes to waypoints published by A* path planner, implements minimum
    snap spline trajectories through waypoints, and publishes path and
    derivatives for trajectory follower
    """
    def __init__(self):
        self.setup_trajectory_generator()
        self.armed = False
        self.start_offset = 0.0  # time offset from when system is armed

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

        self.desired_speed = rospy.get_param(
            'trajectory_generator/desired_speed')
        self.time_scale = rospy.get_param('trajectory_generator/time_scale')
        self.kr = rospy.get_param('trajectory_generator/kr')

        self.waypoints = []
        self.pos_trajectory = None
        self.yaw_trajectory = None

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
        self.traj_msg.layout.dim[1].size = self.kr+1
        self.traj_msg.layout.dim[0].stride = Nr*(self.kr+1)
        self.traj_msg.layout.dim[1].stride = 1
        self.traj_msg.layout.data_offset = 0

    @staticmethod
    def run():
        """This function runs until ROS node is killed"""
        rospy.spin()

    def setup_trajectory_generator(self):
        """Setup position and yaw trajectory generators"""

        basis_type = rospy.get_param('trajectory_generator/basis_type')
        pos_basis_degree = rospy.get_param(
            'trajectory_generator/pos_basis_degree')
        yaw_basis_degree = rospy.get_param(
            'trajectory_generator/pos_basis_degree')

        if basis_type.lower() == 'standard':
            basis_class = SBM
        else:
            print("[Trajectory Node] - Basis type not recognized!")
            raise ValueError

        self.num_waypoints = rospy.get_param(
            'trajectory_generator/num_waypoints')
        self.prepend_current_state = rospy.get_param(
            'trajectory_generator/prepend_current_state')
        if self.prepend_current_state:
            self.num_waypoints += 1

        pos_continuity_derivative = rospy.get_param(
            'trajectory_generator/pos_continuity_derivative')
        yaw_continuity_derivative = rospy.get_param(
            'trajectory_generator/yaw_continuity_derivative')

        pos_smoothness_derivative = rospy.get_param(
            'trajectory_generator/pos_smoothness_derivative')
        yaw_smoothness_derivative = rospy.get_param(
            'trajectory_generator/yaw_smoothness_derivative')

        pos_knot_con_structure_high_d = rospy.get_param(
            'trajectory_generator/pos_knot_con_structure_high_d')

        # Always constrain all waypoints to be at knots
        pos_knot_con_structure_0_d = [{'deriv': 0, 'knot_list': 'all'}]

        if self.prepend_current_state:
            pos_knot_con_structure_high_d = \
                self.modify_high_d_structure_for_prepend_current(
                    pos_knot_con_structure_high_d)

        # Append desired structure for higher derivatives
        pos_knot_con_structure = \
            pos_knot_con_structure_0_d + pos_knot_con_structure_high_d

        # Create spline generators
        self.pos_traj_generator = PTG(self.num_waypoints - 1,
                                      pos_basis_degree + 1,
                                      basis_class,
                                      pos_continuity_derivative,
                                      pos_smoothness_derivative,
                                      pos_knot_con_structure)

        yaw_knot_con_structure_high_d = rospy.get_param(
            'trajectory_generator/yaw_knot_con_structure_high_d')

        if self.prepend_current_state:
            yaw_knot_con_structure_high_d = \
                self.modify_high_d_structure_for_prepend_current(
                    yaw_knot_con_structure_high_d)

        yaw_knot_con_structure_0_d = [{'deriv': 0, 'knot_list': 'all'}]
        yaw_knot_con_structure = \
            yaw_knot_con_structure_0_d + yaw_knot_con_structure_high_d
        self.yaw_traj_generator = PTG(self.num_waypoints - 1,
                                      yaw_basis_degree + 1,
                                      basis_class,
                                      yaw_continuity_derivative,
                                      yaw_smoothness_derivative,
                                      yaw_knot_con_structure)

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

        # only publish trajectory if ROSFlight is armed
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

        # the first knot point is our current position
        waypoints.append(self.current_position)
        # the rest come from the message
        for pose in waypoints_msgs:
            waypoints.append(
                np.array([pose.position.x, pose.position.y, pose.position.z]))

        self.create_trajectory(waypoints)

    def create_trajectory(self, waypoints):
        """
        Generate a new trajectory after receiving new waypoints
        """
        if self.prepend_current_state:
            waypoints = [self.current_position] + waypoints
        waypoints = np.array(waypoints)
        if waypoints.shape[0] != self.num_waypoints:
            rospy.logerr("Invalid number of waypoints")
            return

        kcv_x_high_d = rospy.get_param(
            'trajectory_generator/pos_x_con_value_high_d')
        kcv_y_high_d = rospy.get_param(
            'trajectory_generator/pos_y_con_value_high_d')
        kcv_z_high_d = rospy.get_param(
            'trajectory_generator/pos_z_con_value_high_d')

        if self.prepend_current_state:
            kcv_x_high_d = self.modify_high_d_value_for_prepend_current(
                kcv_x_high_d, self.current_velocity[0])
            kcv_y_high_d = self.modify_high_d_value_for_prepend_current(
                kcv_y_high_d, self.current_velocity[1])
            kcv_z_high_d = self.modify_high_d_value_for_prepend_current(
                kcv_z_high_d, self.current_velocity[2])

        kcv_x_0_d = [{'deriv': 0, 'value_list': waypoints[:, 0]}]
        kcv_x = kcv_x_0_d + kcv_x_high_d

        kcv_y_0_d = [{'deriv': 0, 'value_list': waypoints[:, 1]}]
        kcv_y = kcv_y_0_d + kcv_y_high_d

        kcv_z_0_d = [{'deriv': 0, 'value_list': waypoints[:, 2]}]
        kcv_z = kcv_z_0_d + kcv_z_high_d

        self.pos_trajectory = (self.pos_traj_generator
                               .generate_trajectory([kcv_x, kcv_y, kcv_z]))
        self.pos_trajectory.time_scale = self.time_scale

        # get velocities at each knot point to calculate desired heading
        vel_list = []
        for i in range(self.num_waypoints):
            vel_list.append(self.pos_trajectory.eval(i/self.time_scale, 1))

        yaw_points = mt.velocities_to_headings(vel_list, self.current_yaw)

        kcv_yaw_high_d = rospy.get_param(
            'trajectory_generator/yaw_con_value_high_d')
        if self.prepend_current_state:
            kcv_yaw_high_d = self.modify_high_d_value_for_prepend_current(
                kcv_yaw_high_d, self.current_yaw_rate)

        kcv_yaw_0_d = [{'deriv': 0, 'value_list': yaw_points}]
        kcv_yaw = kcv_yaw_0_d + kcv_yaw_high_d

        self.yaw_trajectory = (self.yaw_traj_generator
                               .generate_trajectory([kcv_yaw]))
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

    @staticmethod
    def modify_high_d_structure_for_prepend_current(knot_con_structure_high_d):
        found_vel = False
        for i in range(len(knot_con_structure_high_d)):
            ks_deriv_dict = knot_con_structure_high_d[i]
            deriv_kl = np.array(ks_deriv_dict['knot_list'])
            # shift all positive indices
            deriv_kl[deriv_kl >= 0] += 1
            if ks_deriv_dict['deriv'] == 1:
                deriv_kl = np.insert(deriv_kl, 0, 0)
                found_vel = True
            knot_con_structure_high_d[i]['knot_list'] = deriv_kl

        if found_vel is False:
            # no existing velocity constraints
            knot_con_structure_high_d.append({'deriv': 1, 'knot_list': [0]})

        return knot_con_structure_high_d

    @staticmethod
    def modify_high_d_value_for_prepend_current(
            knot_con_value_high_d,
            vel_value):
        kv_sorted = sorted(knot_con_value_high_d, key=lambda k: k['deriv'])
        if len(kv_sorted) > 0 and kv_sorted[0]['deriv'] == 1:
            kv_sorted[0]['value_list'].insert(0, vel_value)
        else:
            # no velocity in knot_con_value_high_d
            kv_sorted.append({'deriv': 1, 'value_list': [vel_value]})
        return kv_sorted



if __name__ == '__main__':
    rospy.init_node('spline_trajectory_generator', anonymous=True)
    try:
        ros_node = SplineTrajNode()
        ros_node.run()
    except rospy.ROSInterruptException:
        pass

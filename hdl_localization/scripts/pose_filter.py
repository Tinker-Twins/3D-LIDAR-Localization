#!/usr/bin/env python3

# Copyright (c) 2024, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

################################################################################

class PoseFilter:
    def __init__(self):
        self.init_pose = None # Initial pose (w.r.t. map frame)
        self.odom_pose = None # Estimated pose from localization (w.r.t. map frame)
        self.tf_broadcaster = None
        self.filtered_pose_msg = None
        self.filtered_pose_pub = None
        self.x_offset = 5.000
        self.y_offset = -20.0

    def init_pose_callback(self, msg):
        self.init_pose = msg.pose.pose

    def localization_callback(self, msg):
        self.odom_pose = msg.pose.pose
        
    def filter_pose(self):
        ##############################################################################
        # map_T_odom
        ##############################################################################
        # print(self.init_pose)
        pos_x = self.init_pose.position.x
        pos_y = self.init_pose.position.y
        rot_z = euler_from_quaternion(np.array([self.init_pose.orientation.x,
                                                self.init_pose.orientation.y,
                                                self.init_pose.orientation.z,
                                                self.init_pose.orientation.w]))[2]
        map_T_odom = np.matrix([[np.cos(rot_z), -np.sin(rot_z), pos_x],
                                [np.sin(rot_z), np.cos(rot_z), pos_y],
                                [0, 0, 1]])
        # print(np.round(map_T_odom, 3))

        ##############################################################################
        # map_T_velodyne
        ##############################################################################
        # print(self.odom_pose)
        pos_x = self.odom_pose.position.x
        pos_y = self.odom_pose.position.y
        rot_z = euler_from_quaternion(np.array([self.odom_pose.orientation.x,
                                                self.odom_pose.orientation.y,
                                                self.odom_pose.orientation.z,
                                                self.odom_pose.orientation.w]))[2]
        # print(pos_x, pos_y, rot_z)
        map_T_velodyne = np.matrix([[np.cos(rot_z), -np.sin(rot_z), pos_x],
                                    [np.sin(rot_z), np.cos(rot_z), pos_y],
                                    [0, 0, 1]])
        # print(np.round(map_T_velodyne, 3))
        
        ##############################################################################
        # odom_T_velodyne
        ##############################################################################
        odom_T_velodyne = np.matmul(map_T_odom.I, map_T_velodyne)
        # print(np.round(odom_T_velodyne, 3))

        # Generate and publish `Pose2D` message
        
        self.filtered_pose_msg.x = odom_T_velodyne.item((0, 2)) + self.x_offset
        self.filtered_pose_msg.y = odom_T_velodyne.item((1, 2)) + self.y_offset
        self.filtered_pose_msg.theta = np.arctan2(odom_T_velodyne.item((1, 0)), odom_T_velodyne.item((1, 1)))
        self.filtered_pose_pub.publish(self.filtered_pose_msg)

        ##############################################################################
        # tf
        ##############################################################################
        # Broadcast transform of `odom` frame w.r.t. `map` frame
        self.tf_broadcaster.sendTransform((self.init_pose.position.x, self.init_pose.position.y, self.init_pose.position.z),
                                          (self.init_pose.orientation.x, self.init_pose.orientation.y, self.init_pose.orientation.z, self.init_pose.orientation.w),
                                          rospy.Time.now(), 'odom', 'map')

################################################################################

if __name__ == '__main__':
    rospy.init_node('pose_filter')

    # PoseFilter class instance
    filter_odom_node = PoseFilter()

    # Transform broadcaster
    filter_odom_node.tf_broadcaster = tf.TransformBroadcaster()

    # Message
    filter_odom_node.filtered_pose_msg = Pose2D()

    # Subscribers
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, filter_odom_node.init_pose_callback)
    rospy.Subscriber('/odom', Odometry, filter_odom_node.localization_callback)

    # Publisher
    filter_odom_node.filtered_pose_pub = rospy.Publisher('/odom_pose', Pose2D, queue_size = 10)

    # Wait for valid messages to ensure proper state initialization
    rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
    rospy.wait_for_message('/odom', Odometry)

    # ROS rate
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            filter_odom_node.filter_pose()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

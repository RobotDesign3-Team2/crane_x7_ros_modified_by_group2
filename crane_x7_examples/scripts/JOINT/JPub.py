#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 RT Corporation
#
# Licensed under the RT Corporation NON-COMMERCIAL LICENSE.
# Please see https://github.com/rt-net/crane_x7_ros/blob/master/LICENSE
# for detail.

import rospy
import moveit_commander
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node("JPub")
    pub = rospy.Publisher('xaxis', Float64, queue_size=1)
    pub2 = rospy.Publisher('yaxis', Float64, queue_size=1)
#    robot = moveit_commander.RobotCommander()
   # arm = moveit_commander.MoveGroupCommander("arm")
   # arm.set_max_velocity_scaling_factor(0.1)
  #  gripper = moveit_commander.MoveGroupCommander("gripper")
    rate = rospy.Rate(10)
    x = 0.2
    y = -0.18
    while not rospy.is_shutdown():
        pub.publish(x)
        pub2.publish(y)
        rate.sleep()

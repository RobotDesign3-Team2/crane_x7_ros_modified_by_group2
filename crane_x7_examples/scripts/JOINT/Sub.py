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

def callback(message):
    rospy.loginfo("get message! [%f]", message.data) # ターミナルへの表示

if __name__ == '__main__':
    rospy.init_node('Sub')
    sub = rospy.Subscriber('count_up', Float64, callback) # chatterというTopicを受信！受信したら上で定義したcallback関数を呼ぶ
    rospy.spin()
    

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
#～～～～～～～～～～～～～～～～～～～～～～～～～～～～～～～～
from std_msgs.msg import Float64#Flont64を使うと宣言

if __name__ == '__main__':
    rospy.init_node("JPub")#ノードの名前
    pub = rospy.Publisher('red_x', Float64, queue_size=1)#送るデータの型指定的な
    pub2 = rospy.Publisher('red_y', Float64, queue_size=1)#送るデータの型指定的な
    rate = rospy.Rate(10)#送る周波数を決めている、センサの場合いらない可能性
    x = 400
    y = 300
    while not rospy.is_shutdown():
        pub.publish(x)#x座標を送る
        pub2.publish(y)#y座標を送る
        rate.sleep()#スリープ時間

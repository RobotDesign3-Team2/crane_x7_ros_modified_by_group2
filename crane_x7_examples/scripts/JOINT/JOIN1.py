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
import csv

def follow_circular_path(
    fl, radius=0.1,
    num_of_waypoints=30, repeat=1,
    eef_step=0.0001, jump_threshold=0.0, avoid_collisions=True): #eff_step = アームのスピード変更 jump_threshold(ジャンプしきい値) = 不明 avoid_collisions =衝突しないようにするか、しないか
    fl = ["KO.csv","RO.csv"]
    way_points = []
    q = quaternion_from_euler(3.14/2.0, 0.0, 3.14/2.0)
    target_orientation = Quaternion(q[0], q[1], q[2], q[3])

    for FL in fl:
        with open(FL) as f:
	    for row in csv.reader(f, quoting=csv.QUOTE_NONNUMERIC):
	        arm.set_max_velocity_scaling_factor(0.1)
                target_pose = Pose()
                target_pose.position.x = row[0]
                target_pose.position.y = row[1]
                target_pose.position.z = row[2]
                target_pose.orientation = target_orientation
                way_points.append(target_pose)

    #考察この関数は、for文でループして動かす値を設定してからpath, fractionにデータを収め　arm,pathで収めたデータを動かしているのではないか
    path, fraction = arm.compute_cartesian_path(way_points, eef_step, jump_threshold, avoid_collisions)
    arm.execute(path)

def main():

    fl = ["KO.csv","RO.csv"]

    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    arm.set_named_target("home")
    arm.go()

    # 座標(x=0.3, y=0.0, z=0.1)を中心に、XY平面上に半径0.1 mの円を3回描くように手先を動かす
    follow_circular_path(fl, radius=0.1, repeat=3)

    arm.set_named_target("vertical")
    arm.go()

if __name__ == '__main__':
    rospy.init_node("cartesian_path_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

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
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import rosnode
from tf.transformations import quaternion_from_euler
import csv

def move_max_velocity(value):#速度調節関数
    arm.set_max_velocity_scaling_factor(value)#速度を調整する部分()の中に値を入れると変更出来る

def follow_circular_path(
    fl, radius=0.1,
    num_of_waypoints=30, repeat=1,
    eef_step=0.0001, jump_threshold=0.0, avoid_collisions=True): #eff_step = アームのスピード変更 jump_threshold(ジャンプしきい値) = 不明 avoid_collisions =衝突しないようにするか、しないか
    way_points = []
    q = quaternion_from_euler(3.14/2.0, 0.0, 3.14/2.0)
    target_orientation = Quaternion(q[0], q[1], q[2], q[3])

   
    with open(fl) as f:
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

def follow_circular_path2(
    radius=0.1,
    num_of_waypoints=30, repeat=1,
    eef_step=0.0001, jump_threshold=0.0, avoid_collisions=True): #eff_step = アームのスピード変更 jump_threshold(ジャンプしきい値) = 不明 avoid_collisions =衝突しないようにするか、しな>    いか
    way_points = []
    q = quaternion_from_euler(3.14/2.0, 0.0, 3.14/2.0)
    target_orientation = Quaternion(q[0], q[1], q[2], q[3])
    with open("Nura.csv") as f:
        for row in csv.reader(f, quoting=csv.QUOTE_NONNUMERIC):
            arm.set_max_velocity_scaling_factor(0.1)
            target_pose = Pose()
            target_pose.position.x = point1.x
            target_pose.position.y = point1.y
            target_pose.position.z = row[0]
            target_pose.orientation = target_orientation
            way_points.append(target_pose)
 
    path, fraction = arm.compute_cartesian_path(way_points, eef_step, jump_threshold, avoid_collisions)
    arm.execute(path)

def follow_circular_path3(
    Rr, radius=0.1,
    num_of_waypoints=30, repeat=1,
    eef_step=0.001, jump_threshold=0.0, avoid_collisions=True): #eff_step = アームのスピード変更 jump_threshold(ジ>    ャンプしきい値) = 不明 avoid_collisions =衝突しないようにするか、しな>    いか
    way_points = []
    q = quaternion_from_euler(Rr, 0.0, 3.14/2)
    target_orientation = Quaternion(q[0], q[1], q[2], q[3])

    arm.set_max_velocity_scaling_factor(0.1)
    target_pose = Pose()
    target_pose.position.x = point1.x
    target_pose.position.y = point1.y
    target_pose.position.z = point1.z
    target_pose.orientation = target_orientation
    way_points.append(target_pose)
  
    arm.set_max_velocity_scaling_factor(0.1)
    target_pose = Pose()
    target_pose.position.x = point2.x
    target_pose.position.y = point2.y
    target_pose.position.z = point2.z
    target_pose.orientation = target_orientation
    way_points.append(target_pose)

    path, fraction = arm.compute_cartesian_path(way_points, eef_step, jump_threshold, avoid_collisions)  
    arm.execute(path)
	
def move_arm(Rr):#アームの動き関数 move_arm(0.5, 0.5, 0.1 ,0.1)などの    ように使用
   # move_max_velocity(move)#速度調整関数
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = point1.x#x代入
    target_pose.position.y = point1.y#y代入
    target_pose.position.z = 0.3#z代入
    q = quaternion_from_euler(Rr, 0.0, 3.14/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

def Nura():
    arm.set_named_target("home") #homeに戻らないと挙動がおかしくなるので
    arm.go()
    follow_circular_path2()#上下移動
    rospy.sleep(4.0)#水滴落とし
    follow_circular_path3(Rr=3.14/2)#文字書き初期位置に移動

def main():
    fl = ["REI.csv","RO.csv"] 
    He = ["SEARCH.csv"]

    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()
    move_max_velocity(0.5)
    arm.set_named_target("home")
    arm.go()
     
#    follow_circular_path3(Rr=3.14/2) 
    move_arm(3.14) 
   
    rospy.sleep(5.0)
#検知時間
    move_arm(3.14/2)

    rospy.sleep(5.0)#筆を掴む	    
    gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()
    rospy.sleep(1.0)
    move_max_velocity(0.2)
#水に濡らす
    Nura()
    #文字を書く（令和）
    follow_circular_path(fl[0])
    Nura()
    follow_circular_path(fl[1])   
    move_max_velocity(0.5)
    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':
    rospy.init_node("JOINT1")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    class point1:
         x=0.2
         y=-0.18
         z=0.3
         he=1
    
    class point2:
         x=0.2
         y=0
         z=0.3

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

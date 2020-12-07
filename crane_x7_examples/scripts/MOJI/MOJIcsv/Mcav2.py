#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

def main():
    fl = ["KO.csv","RO.csv","NA.csv"] #読み込むcsvファイル
    fl2 = "NURA.csv"
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    def move_max_velocity(value = 0.2):#速度調節関数
        arm.set_max_velocity_scaling_factor(value)
    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    def move_arm(pos_x, pos_y, pos_z, move):#アームの動き関数 
	    move_max_velocity(move)
	    target_pose = geometry_msgs.msg.Pose()
	    target_pose.position.x = pos_x
	    target_pose.position.y = pos_y
	    target_pose.position.z = pos_z
	    q = quaternion_from_euler(3.14/2.0, 0.0, 3.14/2.0)  # 上方から掴みに行く場合
	    target_pose.orientation.x = q[0]
	    target_pose.orientation.y = q[1]
	    target_pose.orientation.z = q[2]
	    target_pose.orientation.w = q[3]
	    arm.set_pose_target(target_pose)  # 目標ポーズ設定
	    arm.go()  # 実行
	    print("位置")
	    print(pos_x, pos_y, pos_z)
	    print("速度")
	    print(move)

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"vertical"の姿勢にする
    print("vertical")
    move_max_velocity(0.2)
    arm.set_named_target("vertical")
    arm.go()
	
    move_arm(0.38, 0.024, 0.2, 0.2)

    print("筆を掴ませる待ち時間")
    rospy.sleep(5.0) #ここで筆を掴む
    
    gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()

    #筆に水をつける
    import csv
    with open(fl2) as f: # |
	for row in csv.reader(f, quoting=csv.QUOTE_NONNUMERIC):#ファイル
	    rospy.sleep(1.0)
	    move_arm(row[0],row[1],row[2], 0.05) #move_arm(X,Y,Z, アームのスピード)

    rospy.sleep(8.0)
    move_arm(0.386,0.024,0.2, 0.05)	

    #文字を書く(コロナ)
    for FL in fl:#ファイルの数ループ
	import csv #csvファイルを開く（座標が入っている）
	with open(FL) as f: # |
	    for row in csv.reader(f, quoting=csv.QUOTE_NONNUMERIC):
		move_arm(row[0],row[1],row[2], 0.02) #move_arm(X,Y,Z, アームのスピード)
		rx = row[0]
		ry = row[1]
		rz = row[2]
	rospy.sleep(1.0) 

    move_arm(rx,ry,rz+0.05, 0.2)
    move_arm(rx,ry,rz+0.1, 0.2)
    print("vertical")
    move_max_velocity(0.2)
    arm.set_named_target("vertical")
    arm.go()

if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

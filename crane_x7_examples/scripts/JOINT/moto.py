#!/usr/bin/env python
## coding: UTF-8

import rospy
from std_msgs.msg import Float64 # ROS通信で文字列を取得できるようにstd_msgsというパッケージからStringという型を取得
from datetime import datetime

cot = 10
rospy.init_node('moto') # ノードの生成
pub = rospy.Publisher('chatter', String, queue_size=10) # chatterという名前のTopicを生成し型やらを定義
rate = rospy.Rate(10) # 10Hzで動かすrateというクラスを生成
print("Conection started...")
while not rospy.is_shutdown():
    dou = String # Stringというクラスで送信するメッセージ、"hello_str"を生成
   # timestamp = rospy.get_time()
   # time = datetime.fromtimestamp(timestamp) # ここらへんはROSと関係ないです
    dou.x = cot # 内容の書き込み
    dou.y = 100
    pub.publish(dou) # hello_strを送信！
    rate.sleep() # 先程定義したrateをここで動かす

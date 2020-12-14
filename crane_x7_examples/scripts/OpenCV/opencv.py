#!/usr/bin/env python
#-*- coding:utf-8 -*-
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

        # 赤色の検出
    def callback(self, data):
        pub = rospy.Publisher("red_size", Float64MultiArray, queue_size=1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # HSV色空間に変換
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 赤色のHSVの値域1
        hsv_min = np.array([0,64,0])
        hsv_max = np.array([30,255,255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        # 赤色のHSVの値域2
        hsv_min = np.array([150,64,0])
        hsv_max = np.array([179,255,255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        # 赤色領域のマスク（255：赤色、0：赤色以外）    
        mask = mask1 + mask2

        # 画像配列のビット毎の倫理席。マスク画像だけが抽出される。                               
        cv_image2  = cv2.bitwise_and(cv_image, cv_image, mask = mask)

        labels,contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        areas = np.array(list(map(cv2.contourArea,contours)))

        for i in range(0, len(contours)):
            if len(contours[i]) > 0:

                # remove small objects
                if cv2.contourArea(contours[i]) < 500:
                    continue

                rect = contours[i]
                x, y, w, h = cv2.boundingRect(rect)
                if len(areas) == 0 or np.max(areas) / (h*w) < 0.005:
                    # 見つからなかったらNoneを返す
                    print("the area is too small")
                    return None
                elif np.max(areas) >= 1000:
                # 面積が最大の塊の重心を計算し返す
                    max_idx = np.argmax(areas)
                    max_area = areas[max_idx]
                    result = cv2.moments(contours[max_idx])
                    x = float(result["m10"]/result["m00"])
                    y = float(result["m01"]/result["m00"])
                    size = x,y
                    print(size)
                    array_forPublish = Float64MultiArray(data=size)
                    pub.publish(array_forPublish)
                else:
                    print("the area is too small")
        #ウインドウのサイズを変更                                                               
        cv_half_image = cv2.resize(cv_image,   (0,0),fx=0.5, fy=0.5)
        cv_half_image = cv2.resize(cv_image,   (0,0),fx=0.5, fy=0.5)

      # ウインドウ表示                                                                         
        cv2.imshow("Origin Image", cv_image)
        cv2.imshow("Red Image", cv_image2)
        cv2.waitKey(3)
        

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()
"""
def main():
# webカメラを扱うオブジェクトを取得
    cap = cv2.VideoCapture(0)
    flag = True
    o_pos = 0,0
    while flag:
        ret,frame = cap.read()

        if ret is False:
            print("cannot read image")
            continue
        pos = detect_red_color(frame)

        print(pos)
        if pos == o_pos:
            flag = False
        o_pos = pos
"""
if __name__ == '__main__':
    sub = rospy.Subscriber("/red_color", Image, main)
    main(sys.argv)
    pub = rospy.Publisher("red_rocate", Float64MultiArray, queue_size=1)
    rospy.spin()

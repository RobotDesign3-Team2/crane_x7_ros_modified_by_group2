#-*- coding:utf-8 -*-
import cv2
import numpy as np

# 赤色の検出
def detect_red_color(img):
    h,w,c = img.shape
    # HSV色空間に変換
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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

    _,contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    areas = np.array(list(map(cv2.contourArea,contours)))

    if len(areas) == 0 or np.max(areas) / (h*w) < 0.005:
        # 見つからなかったらNoneを返す
        print("the area is too small")
        return None
    else:
        # 面積が最大の塊の重心を計算し返す
        max_idx = np.argmax(areas)
        max_area = areas[max_idx]
        result = cv2.moments(contours[max_idx])
        x = int(result["m10"]/result["m00"])
        y = int(result["m01"]/result["m00"])
        return (x,y)
"""
def test():
# 入力画像の読み込み
    img = cv2.imread("test1.png")

# 色検出
    pos= detect_red_color(img)
        if pos is not None:
            cv2.circle(img,pos,10,(0,255,255),-1)

# 結果を出力
    cv2.imwrite("result.png",img)
"""

def main():
    # webカメラを扱うオブジェクトを取得
    cap = cv2.VideoCapture(0)
    while True:
        ret,frame = cap.read()

        if ret is False:
            print("cannot read image")
            continue
        pos = detect_red_color(frame)
        print(pos)

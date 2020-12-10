import cv2
import numpy as np

LOW_COLOR = np.array([100, 75, 75])
HIGH_COLOR = np.array([140, 255, 255])
# 赤色のHSVの値域1
hsv_min = np.array([0,64,0])
hsv_max = np.array([30,255,255])

# 赤色のHSVの値域2
#hsv_min2 = np.array([150,64,0])
#hsv_max2 = np.array([179,255,255])

# 赤色領域のマスク（255：赤色、0：赤色以外）    
#mask = mask1 + mask2

# 抽出する赤色の塊のしきい値
AREA_RATIO_THRESHOLD = 0.005

def find_specific_color(frame,AREA_RATIO_THRESHOLD,LOW_COLOR,HIGH_COLOR):
    """
    指定した範囲の色の物体の座標を取得する関数
    frame: 画像
    AREA_RATIO_THRESHOLD: area_ratio未満の塊は無視する
    LOW_COLOR: 抽出する色の下限(h,s,v)
    HIGH_COLOR: 抽出する色の上限(h,s,v)
    """
    # 高さ，幅，チャンネル数
    h,w,c = frame.shape

    # hsv色空間に変換
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    # 色を抽出する
    mask = cv2.inRange(hsv, hsv_min, hsv_max)
#    ex_img = cv2.inRange(hsv,LOW_COLOR,HIGH_COLOR)

    # 輪郭抽出
    _,contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    # 面積を計算
    areas = np.array(list(map(cv2.contourArea,contours)))

    if len(areas) == 0 or np.max(areas) / (h*w) < AREA_RATIO_THRESHOLD:
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
    img = cv2.imread("sample.jpg")

    # 位置を抽出
    pos = find_specific_color(
        img,
        AREA_RATIO_THRESHOLD,
        LOW_COLOR,
        HIGH_COLOR
    )

    if pos is not None:
        cv2.circle(img,pos,10,(0,0,255),-1)
    
    cv2.imwrite("result.jpg",img)
"""
def main():
    # webカメラを扱うオブジェクトを取得
    cap = cv2.VideoCapture(0)


    while True:
        ret,frame = cap.read()

        if ret is False:
            print("cannot read image")
            continue

        # 位置を抽出
        pos = find_specific_color(
            frame,
            AREA_RATIO_THRESHOLD,
            hsv_min,
            hsv_max
        )


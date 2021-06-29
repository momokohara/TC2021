#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("contourimg_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("segmentated_image",Image,self.callback)


    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("debug", frame)
        input_image = np.array(frame, dtype=np.uint8)
        rects = self.process_image(input_image)

        print(frame)
        cv2.imshow("ros_img", frame)
        print(rects)
        cv2.waitKey(1)
    

    def process_image(self, image):
        #しきい値の設定(ここでは紫を抽出）
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        color_min = np.array([237,170,220])
        color_max = np.array([237,170,220])

        #マスク画像を生成
        mask = cv2.inRange(image, color_min, color_max);
        print(mask)
        
        display = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("hsv filter", display)

    
    #mask画像から輪郭抽出
        image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #print(image)    
        #print(contours)
        #輪郭面積最大color_image.flags.writeable = Trueの物体の情報を取得
        for cnt in contours:
            area = cv2.contourArea(cnt)
            areaMin = 1
            if area > areaMin:

                # 輪郭の描画
                maxC = cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
                #result = cv2.bitwise_and(img, img, mask = bin_img)

                #重心に円を描画
                mu = cv2.moments(maxC)
                x, y = int(mu["m10"]/mu["m00"]), int(mu["m01"]/mu["m00"])
                contour_img = cv2.circle(maxC, (x, y), 4, 100, 2, 4)

                # ウインドウ表示
                cv2.imshow("Result Image", contour_img)
                cv2.waitKey(3)
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(contour_img, "bgr8"))
                except CvBridgeError as e:
                    print(e)
#            else:
#               print(contour_img)


if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


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
        self.pub = rospy.Publisher("Momentimg_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("segmentated_image",Image,self.callback)


    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        input_image = np.array(frame, dtype=np.uint8)
        rects = self.process_image(input_image)

        print(rects)
        cv2.waitKey(1)
    

    def process_image(self, image):
        #しきい値の設定(ここでは紫を抽出）
        color_min = np.array([192,0,128])
        color_max = np.array([192,0,128])

        #マスク画像を生成
        mask = cv2.inRange(image, color_min, color_max);
        
        display = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("hsv filter", display)

    
        #mask画像から輪郭抽出
        input_image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        #輪郭がない場合
        if len(contours) == 0:
            print("contour is empty")
            cv2.waitKey(3)
            try:
                self.pub.publish(self.bridge.cv2_to_imgmsg(mask, "bgr8"))
            except CvBridgeError as e:
                print(e)
            
            return mask

        
        #輪郭がある場合
        else:

            max_cnt = max(contours, key=lambda x: cv2.contourArea(x))  
              
            # 輪郭の描画
            maxC = cv2.drawContours(input_image, max_cnt, -1, (0, 255, 0), 3)
            cv2.imshow("drawed contour", maxC)
            
            #重心に円を描画
            mu = cv2.moments(maxC)
            x, y = int(mu["m10"]/mu["m00"]), int(mu["m01"]/mu["m00"])
            moment_img = cv2.circle(image, (x, y), 4, 100, 2, 4)

            # ウインドウ表示
            cv2.imshow("Moment Image", moment_img)
            cv2.waitKey(3)
            try:
                imageMsg = self.bridge.cv2_to_imgmsg(moment_img, "bgr8")
                self.pub.publish(imageMsg)
            except CvBridgeError as e:
                print(e)
            return moment_img


if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


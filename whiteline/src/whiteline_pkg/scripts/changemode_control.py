#!/usr/bin/env python
# coding: utf-8


import rospy
import cv2
import math
import numpy as np
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
bool ch_flag


class ObjectTracker():

    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_image = None
        self._object_pixels = 0  # Maximum area detected in the current image[pixel]
        self._object_pixels_default = 0  # Maximum area detected from the first image[pixel]
        self._point_of_centroid = None

        self._pub_binary_image = rospy.Publisher("binary", Image, queue_size=1)
        self._pub_pbject_image = rospy.Publisher("object", Image, queue_size=1)
        self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._sub_image = rospy.Subscriber("/segmentated_image", Image, self._image_callback)
    
    def _image_callback(self, img):
        try:
            self._captured_image = self._cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def _pixels(self, cv_image):
        return cv_image.shape[0] * cv_image.shape[1]

    def _object_is_detected(self):
        # Lower limit of the ratio of the detected area to the screen.
        # Object tracking is not performed below this ratio.
        LOWER_LIMIT = 0.01

        if self._captured_image is not None:
            object_per_image = self._object_pixels / self._pixels(self._captured_image)
            return object_per_image > LOWER_LIMIT
        else:
            return False

    def _object_pixels_ratio(self):
        if self._captured_image is not None:
            diff_pixels = self._object_pixels - self._object_pixels_default
            return diff_pixels / self._pixels(self._captured_image)
        else:
            return 0

    def _object_is_smaller_than_default(self):
        return self._object_pixels_ratio() < -0.01

    def _set_color(self):
        min_bgr = np.array([192, 0, 128])
        max_bgr = np.array([192, 0, 128])
        return min_bgr, max_bgr

    def _extract_object_in_binary(self, cv_image):
        if cv_image is None:
            return None

        min_bgr, max_bgr = self._set_color()

        binary = cv2.inRange(cv_image, min_bgr, max_bgr)
        # Morphology
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        return binary

    def _calibrate_object_pixels_default(self):
        if self._object_pixels_default == 0 and self._object_pixels != 0:
            self._object_pixels_default = self._object_pixels

    def _extract_biggest_contour(self, binary_img):
        biggest_contour_index = False
        biggest_contour_area = 0
        _, contours, hierarchy = cv2.findContours(
            binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if biggest_contour_area < area:
                biggest_contour_area = area
                biggest_contour_index = i

        if biggest_contour_index is False:
            return False
        else:
            return contours[biggest_contour_index]

    def _calculate_centroid_point(self, contour):
        point = False
        if self._object_is_detected():
            M = cv2.moments(contour)
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])
            point = (centroid_x, centroid_y)

        return point

    def _draw_contour(self, input_image, contour):
        return cv2.drawContours(input_image, [contour], 0, (0, 255, 0), 5)

    def _draw_centroid(self, input_image, point_centroid):
        return cv2.circle(input_image, point_centroid, 15, (255, 0, 0), thickness=-1)

    def _monitor(self, img, pub):
        if img.ndim == 2:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "mono8"))
        elif img.ndim == 3:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            pass

    def _stop_threshold(self):
        stop_threshold = 48
        not_stop_range = self._captured_image.shape[0] - stop_threshold
        return not_stop_range

    def _move_zone(self):
        if self._point_of_centroid[1] <= self._stop_threshold():
            return True

    def _stop_zone(self):
        if self._point_of_centroid[1] > self._stop_threshold():
            return True

    def _rotation_velocity(self):
        VELOCITY = 0.25 * math.pi
        if not self._object_is_detected() or self._point_of_centroid is None or self._stop_zone():
            return 0.0

        half_width = self._captured_image.shape[1] / 2.0
        pos_x_rate = (half_width - self._point_of_centroid[0]) / half_width
        rot_vel = pos_x_rate * VELOCITY
        return rot_vel
    
    def image_processing(self):
        object_image = copy.deepcopy(self._captured_image)
        object_binary_img = self._extract_object_in_binary(self._captured_image)

        if object_binary_img is not None:
            biggest_contour = self._extract_biggest_contour(object_binary_img)
            if biggest_contour is not False:
                self._object_pixels = cv2.contourArea(biggest_contour)
                self._calibrate_object_pixels_default()

                object_image = self._draw_contour(object_image, biggest_contour)

                point = self._calculate_centroid_point(biggest_contour)
                if point is not False:
                    self._point_of_centroid = point
                    object_image = self._draw_centroid(object_image, point)

            self._monitor(object_binary_img, self._pub_binary_image)
            self._monitor(object_image, self._pub_pbject_image)

    def control(self):
        cmd_vel = Twist()
        if self._object_is_detected() and self._point_of_centroid:
            # Move backward and forward by difference from default area
            if self._move_zone():
                cmd_vel.linear.x = 0.1
                print("forward")
            if self._stop_zone():
                cmd_vel.linear.x = 0
                print("stay")
            cmd_vel.angular.z = self._rotation_velocity()
        self._pub_cmdvel.publish(cmd_vel)

    def Callback(data):
        resp = SetBoolResponse()
        req = SetBoolRequest()
        if data.data==True:
            resp.message = "called"
            resp.success = True
            ch_flag = True
        else:
            resp.message = "ready"
            resp.success = False
            ch_flag = False

if __name__ == '__main__':
    rospy.init_node('object_tracking')
    #rospy.sleep(1)
    while not rospy.is_shutodwn():
        srv = rospy.Service('mode_on_off', SetBool, Callback)
        if (ch_flag):
            ot = ObjectTracker()
            rate = rospy.Rate(60)
            ot.control()
            rate.sleep()


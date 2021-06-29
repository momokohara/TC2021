#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

img = cv2.imread("/home/hara/TC2021/script/datasets/datasets_label/frame000000_L.png")

bgr_l = np.array([192, 0, 128])
bgr_u = np.array([192, 0, 128])
bin_img = cv2.inRange(img, bgr_l, bgr_u)

image, contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

maxCont = contours[0]
for c in contours:
    if len(maxCont) < len(c):
        maxCont = c

img_contour = cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
#result = cv2.bitwise_and(img, img, mask = bin_img)

mu = cv2.moments(maxCont)
x, y = int(mu["m10"]/mu["m00"]), int(mu["m01"]/mu["m00"])
a = cv2.circle(img_contour, (x, y), 4, 100, 2, 4)


cv2.imshow("cont_img", a)

cv2.waitKey(0)
cv2.destroyAllWindows()

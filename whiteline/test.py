#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

#画像のパス指定
img = cv2.imread("/home/hara/TC2021/script/datasets/datasets_label/frame000000_L.png")

#白線のBGRの範囲指定
bgr_lower = np.array([192, 0, 128])
bgr_uper = np.array([192, 0, 128])
bin_img = cv2.inRange(img, bgr_lower, bgr_uper)

#mask画像から輪郭抽出
image, contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

#輪郭面積最大の物体の情報を取得
maxCont = contours[0]
for c in contours:
    if len(maxCont) < len(c):
        maxCont = c

#輪郭の描画
img_contour = cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
#result = cv2.bitwise_and(img, img, mask = bin_img)

#重心に円を描画
mu = cv2.moments(maxCont)
x, y = int(mu["m10"]/mu["m00"]), int(mu["m01"]/mu["m00"])
a = cv2.circle(img_contour, (x, y), 4, 100, 2, 4)


cv2.imshow("cont_img", a)
cv2.imwrite("/home/hara/whiteline/cont_img.png", a)

cv2.waitKey(0)
cv2.destroyAllWindows()

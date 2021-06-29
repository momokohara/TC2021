#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import glob

#画像のパス指定
files = glob.glob("/home/hara/TC2021/script/datasets/datasets_label/frame*")

for fname in files:
    bgr = cv2.imread(fname, cv2.IMREAD_COLOR)
    # BGRの範囲指定
    mask = [220, 170, 237]
    after_color = [192, 0, 0]
    bgr[np.where((bgr == mask).all(axis=2))] = after_color
    cv2.imwrite(fname, bgr)
    print(fname)

cv2.destroyAllWindows()

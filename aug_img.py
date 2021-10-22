#  -*- coding: utf-8 -*-

import cv2 
import glob
import os

# パスの設定
from_dir = ("/Users/momokohara/Documents/work/augmentation/")
to_dir = ("/Users/momokohara/Documents/work/augmentation_result/")

# パラメータ設定 
list_saturation = [0.5, 0.7, 1, 1]  # 彩度(Saturation)の倍率
list_value = [1, 1, 0.5, 0.7]  # 明度(Value)の倍率


def augmentation(img, s_magnification, v_magnification):
	input_img = cv2.imread('img') 
	img_hsv = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV) 
	
	img_hsv[:,:,(1)] = img_hsv[:,:,(1)]*s_magnification  # 彩度の計算
	img_hsv[:,:,(2)] = img_hsv[:,:,(2)]*v_magnification  # 明度の計算
	img_bgr = cv2.cvtColor(img_hsv,cv2.COLOR_HSV2BGR)  # 色空間をHSVからBGRに変換

	return img_bgr	

if __name__ == "__main__":
	path = sorted(glob.glob(from_dir + "*.png"))
	# print(path)
	for i in range(4):
		s = list_saturation[i]
		v = list_value[i]
		print("s:" + str(s) + ", v:" + str(v))
		print("________start augmentation : " + str(len(path)) + " files________")
		for img in path:
			rgb_img = cv2.imread(img)
			# print(rgb_img.shape)
			
			img_hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
			img_hsv[:,:,(1)] = img_hsv[:,:,(1)]*s  # 彩度の計算
			img_hsv[:,:,(2)] = img_hsv[:,:,(2)]*v  # 明度の計算
			aug_img = cv2.cvtColor(img_hsv,cv2.COLOR_HSV2BGR)  # 色空間をHSVからBGRに変換	
			basename = os.path.basename(img)

			cv2.imwrite(os.path.join(to_dir, "s" + str(s) + "_v" + str(v) + "_" + basename), aug_img)

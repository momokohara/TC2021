#  -*- coding: utf-8 -*-
import glob
import os
import shutil

# パスの設定
from_dir = ("/Users/momokohara/Documents/work/augmentation/")
to_dir = ("/Users/momokohara/Documents/work/augmentation_result/")

# パラメータ設定
list_saturation = [0.5, 0.7, 1, 1]  # 彩度(Saturation)の倍率
list_value = [1, 1, 0.5, 0.7]  # 明度(Value)の倍率


if __name__ == "__main__":
	path = sorted(glob.glob(from_dir + "*.txt"))
	# print(path)
	for i in range(4):
		s = list_saturation[i]
		v = list_value[i]
		print("s:" + str(s) + ", v:" + str(v))
		print("________start augmentation : " + str(len(path)) + " files________")
		for txt in path:
			shutil.copy(txt, to_dir) 
			basename = os.path.basename(txt)
			
			to_txt = os.path.join(to_dir, basename)
			os.rename(to_txt, to_dir + "s" + str(s) + "_v" + str(v) + "_" + basename)

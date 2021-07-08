#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import os
import sys
import time
from PIL import Image, ImageDraw,ImageFont
import rospy
from spark_carry_object.msg import *

def mean_hsv(img,HSV_value):
	HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	HSV_value[0]+=np.mean(HSV[:, :, 0])
	HSV_value[1]+=np.mean(HSV[:, :, 1])
	HSV_value[2]+=np.mean(HSV[:, :, 2])
	return HSV_value


def hsv_range(HSV_value):
	# 设置HSV颜色值的范围
	H_range = 12
	S_range = 120
	V_range = 120

	lower_H = int(HSV_value[0] - H_range)
	upper_H = int(HSV_value[0] + H_range)

	lower_S = int(HSV_value[1] - S_range)
	upper_S = int(HSV_value[1] + S_range)

	lower_V = int(HSV_value[2] - V_range)
	upper_V = int(HSV_value[2] + V_range)

	if lower_H<0:
		lower_H=0
	if upper_H>180:
		upper_H=180

	if lower_S<50:
		lower_S=50
	if upper_S>255:
		upper_S=255

	if lower_V<50:
		lower_V=50
	if upper_V>255:
		upper_V=255

	lower_HSV = np.array([lower_H, lower_S, lower_V])
	upper_HSV = np.array([upper_H, upper_S, upper_V])
	return lower_HSV, upper_HSV


def test(lower_HSV, upper_HSV, image):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_HSV, upper_HSV)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	mask = cv2.GaussianBlur(mask, (3, 3), 0)
	cv2.putText(mask, 'Done! Press q to exit!', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
				(255, 255, 255), 2, cv2.LINE_AA)
	cv2.imshow("HSV_img", mask)

def arm_init():
	pub1 = rospy.Publisher('position_write_topic', position, queue_size=1)

	r1 = rospy.Rate(1)
	r1.sleep()
	pos = position()
	pos.x = 120
	pos.y = 0
	pos.z = 35
	pub1.publish(pos)
	r1.sleep()

def save_hsv(name, lower_HSV, upper_HSV):

	content = str(lower_HSV[0]) + ',' +str(lower_HSV[1])+ ',' + str(lower_HSV[2]) \
			  + ' ' + str(upper_HSV[0])+ ',' + str(upper_HSV[1])+ ',' + str(upper_HSV[2]) + '\n'
	# 将HSV值写入文件，文件在spark目录下
	if name == 'color_block':
		content = "block_HSV is :" + content
		filename = os.environ['HOME'] + "/color_block_HSV.txt"
	elif name == 'calibration':
		content = "calibration_HSV is :" + content
		filename = os.environ['HOME'] + "/calibration_HSV.txt"

	with open(filename, "w") as f:
		f.write(content)

	print("HSV value has saved in" + filename)
	print(content)

def main():
	name = rospy.get_param("/spark_hsv_detection/color")
	arm_init()
	time.sleep(3)
	HSV_value = [0,0,0]
	count = 0
	capture = cv2.VideoCapture(0)
	# 颜色块中心点上下左右扩大60个像素
	box_w = 60
	# 吸盘上方颜色像素范围
	cali_w = 20
	cali_h = 30
	# 收集300次取平均值
	collect_times = 300
	while True:
		ret, img = capture.read()
		if img is not None:
			# 200次以内先做提醒，将颜色块放在矩形框中
			if count < 200:
				cv2.putText(img, 'please put the color being', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
							(0, 255, 0), 2, cv2.LINE_AA)
				cv2.putText(img, 'tested in rectangle box!', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
							(0, 255, 0), 2, cv2.LINE_AA)
			# 如果在200-500次以内开始收集hsv值，并求出平均值
			elif count > 200 and count < 200+collect_times:
				cv2.putText(img, 'HSV_value is collecting!', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
							(0, 255, 0), 2, cv2.LINE_AA)
				if name == 'color_block':
					frame = img[120:120 + box_w, 300:300 + box_w]
				elif name == 'calibration':
					frame = img[310:310 + cali_h, 355:355 + cali_w]
				HSV_value = mean_hsv(frame, HSV_value)
			# 500次以后开始检测，查看是否有提取到矩形框中颜色的HSV值
			elif count==200+collect_times:
				for i in range(len(HSV_value)):
					HSV_value[i] = HSV_value[i] / collect_times
				lower_HSV, upper_HSV = hsv_range(HSV_value)
				save_hsv(name, lower_HSV, upper_HSV)

			elif count > 200+collect_times:
				test(lower_HSV, upper_HSV, img)

			count += 1
			if name == 'color_block':
				cv2.rectangle(img, (300, 120), (300+box_w, 120+box_w), (0, 255, 0), 3)
			elif name == 'calibration':
				cv2.rectangle(img, (355, 310), (355 + cali_w, 310 + cali_h), (0, 255, 0), 3)
			cv2.imshow("RGB_img", img)
			cv2.waitKey(1)
			if cv2.waitKey(10) == ord('q'):
				break
	cv2.destroyAllWindows()
	return lower_HSV, upper_HSV

if __name__ == '__main__':
	rospy.init_node('hsv_detection', anonymous=True)
	main()
	rospy.spin()

#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import os
#import pandas as pd
from std_msgs.msg import String
from sensor_msgs.msg import Image
from swiftpro.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sklearn.linear_model import LinearRegression

xc = 0
yc = 0

count = 20

index = 0

xarray = np.zeros(count)
yarray = np.zeros(count)
xc_array = np.zeros(count)
yc_array = np.zeros(count)


def image_callback(data):
	global xc, yc
	# change to opencv
	try:
		cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	# change rgb to hsv
	cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)

	# extract blue
	LowerBlue = np.array([100, 90, 80])
	UpperBlue = np.array([130, 255, 255])
	mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)
	cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)

	# gray process
	cv_image4 = cv_image3[:,:,0]

	# smooth and clean noise
	blurred = cv2.blur(cv_image4, (9, 9))
	(_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
	cv_image5 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
	cv_image5 = cv2.erode(cv_image5, None, iterations=4)
	cv_image5 = cv2.dilate(cv_image5, None, iterations=4)

	# detect contour
	cv2.imshow("win2", cv_image5)
	cv2.waitKey(1)
	contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	size = []
	size_max = 0
	for i, c in enumerate(contours):
		rect = cv2.minAreaRect(c)
		box = cv2.cv.BoxPoints(rect)
		box = np.int0(box)
		x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
		y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
		w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
		h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)
		size.append(w * h)
		if size[i] > size_max:
			size_max = size[i]
			index = i
			xc = x_mid
			yc = y_mid

	# cv2.imshow("win1", cv_image1)


def command_callback(data):
	global xc_array
	global yc_array
	global xarray
	global yarray 
	global index
	# s = '' + str(xc) + ' ' + str(yc) + '\n'
	# file_pix = open('thefile.txt', 'a')
	# file_pix.write(s)
	# file_pix.close()
	# print(xc, yc)
	if index < 20:
		
		xc_array[index] = xc
		yc_array[index] = yc
		xarray[index] = 170 + index * 5
		yarray[index] = 200 - index * 10
		print("%d/20,pose x,y: %d,%d. cam x,y: %d,%d" % (index+1, xarray[index], yarray[index], xc, yc))
		# reshape to 2D array for linear regression
		xc_array = xc_array.reshape(-1,1)
		yc_array = yc_array.reshape(-1,1)
		xarray = xarray.reshape(-1,1)
		yarray = yarray.reshape(-1,1)
		index = index + 1
	if index == 20:
		Reg_x_yc = LinearRegression().fit(yc_array, xarray)
		Reg_y_xc = LinearRegression().fit(xc_array, yarray)
		k1 = Reg_x_yc.coef_[0][0]
		b1 = Reg_x_yc.intercept_[0]
		k2 = Reg_y_xc.coef_[0][0]
		b2 = Reg_y_xc.intercept_[0]
		s = '' + str(k1) + ' ' + str(b1) + ' ' + str(k2) + ' ' + str(b2) + '\n'
		filename = os.environ['HOME'] + "/thefile.txt"
		file_pix = open(filename, 'w')
		file_pix.write(s)
		file_pix.close()
		print("Linear Regression for x and yc is :  x = %.5fyc + (%.5f)" % (k1, b1))
		print("Linear Regression for y and xc is :  y = %.5fxc + (%.5f)" % (k2, b2))
		index = 0


def main():
	rospy.init_node('image_converter', anonymous=True)
	sub1 = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
	sub2 = rospy.Subscriber("cali_pix_topic", status, command_callback)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()

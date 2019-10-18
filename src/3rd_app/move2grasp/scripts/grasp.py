#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import random
import ctypes
import roslib
import rospy
import smach
import smach_ros
import threading
import thread
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *


class GraspObject():
    '''
    监听主控，用于物品抓取功能
    '''

    def __init__(self):
        '''
        初始化
        '''

        global xc, yc, xc_prev, yc_prev, found_count
        xc = 0
        yc = 0
        xc_prev = xc
        yc_prev = yc
        found_count = 0
        self.is_found_object = False
        # self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber(
            '/grasp', String, self.grasp_cp, queue_size=1)
        # 发布机械臂位姿
        self.pub1 = rospy.Publisher(
            'position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # 发布机械臂状态
        self.grasp_status_pub = rospy.Publisher(
            'grasp_status', String, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def grasp_cp(self, msg):
        if msg.data == '1':
            # 订阅摄像头话题,对图像信息进行处理
            self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
            self.is_found_object = False
            rate = rospy.Rate(10)
            times=0
            steps=0
            while not self.is_found_object:
                rate.sleep()
                times+=1
                # 转一圈没有发现可抓取物体,退出抓取
                if steps>=5:
                    self.sub.unregister()
                    print("stop grasp\n")
                    status=String()
                    status.data='-1'
                    self.grasp_status_pub.publish(status)
                    return
                # 旋转一定角度扫描是否有可供抓取的物体
                if times>=30:
                    times=0
                    steps+=1
                    self.turn_body()
                    print("not found\n")
            print("unregisting sub\n")
            self.sub.unregister()
            print("unregisted sub\n")
            # 抓取检测到的物体    
            self.grasp()
            status=String()
            status.data='1'
            self.grasp_status_pub.publish(status) 
        if msg.data=='0':
            # 放下物体
            self.is_found_object = False
            self.release_object()
            status=String()
            status.data='0'
            self.grasp_status_pub.publish(status) 

    # 执行抓取
    def grasp(self):
        print("start to grasp\n")
        global xc, yc, found_count
        # stop function

        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        file_pix.close()
        print(s)
        arr=s.split()
        a1=arr[0]
        a2=arr[1]
        a3=arr[2]
        a4=arr[3]
        a = [0]*2
        b = [0]*2
        a[0]=float(a1)
        a[1]=float(a2)
        b[0]=float(a3)
        b[1]=float(a4)
        print('k and b value:',a[0],a[1],b[0],b[1])
        r1 = rospy.Rate(0.095)
        r2 = rospy.Rate(10)
        pos = position()
        # 物体所在坐标+标定误差
        pos.x = a[0] * yc + a[1]
        pos.y = b[0] * xc + b[1]
        pos.z = 20
	    # pos.z = 20
        print("z = 20\n")
        self.pub1.publish(pos)
        r2.sleep()
        # go down -100
        pos.z = -50
        self.pub1.publish(pos)
        print("z = -83\n")
        r2.sleep()

        # 开始吸取物体
        self.pub2.publish(1)
        r2.sleep()

        # 提起物体
        pos.x = 250  #160
        pos.y = 0
        pos.z = 150  #55
        self.pub1.publish(pos)
        r1.sleep()
    # 使用CV检测物体       
    def image_cb(self, data):
        global xc, yc, xc_prev, yc_prev, found_count
        # change to opencv
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('error')

        # change rgb to hsv
        cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)

        # 蓝色物体颜色检测范围
        LowerBlue = np.array([100, 90, 80])
        UpperBlue = np.array([130, 255, 255])
        mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)
        cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)

        # gray process
        cv_image4 = cv_image3[:, :, 0]

        # smooth and clean noise
        blurred = cv2.blur(cv_image4, (9, 9))
        (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        cv_image5 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        cv_image5 = cv2.erode(cv_image5, None, iterations=4)
        cv_image5 = cv2.dilate(cv_image5, None, iterations=4)

        # detect contour
        # cv2.imshow("win1", cv_image1)
        # cv2.imshow("win2", cv_image5)
        # cv2.waitKey(1)
        _, contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # if find contours, pick the biggest box
        if len(contours) > 0:
            size = []
            size_max = 0
            for i, c in enumerate(contours):
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
                w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2) 
                size.append(w * h)
                if size[i] > size_max:
                    size_max = size[i]
                    index = i
                    xc = x_mid
                    yc = y_mid
            # if box is not moving for 20 times
            # print found_count
            if found_count >= 20:
                self.is_found_object = True
            else:
                # if box is not moving
                if abs(xc - xc_prev) <= 2 and abs(yc - yc_prev) <= 2:
                    found_count = found_count + 1
                else:
                    found_count = 0
        else:
            found_count = 0
        xc_prev = xc
        yc_prev = yc
    # 释放物体
    def release_object(self):
        r1 = rospy.Rate(0.15)  # 5s
        r2 = rospy.Rate(1)     # 1s
        pos = position()
        # go forward
        pos.x = 200
        pos.y = 0
        pos.z = -40  #-80
        self.pub1.publish(pos)
        r1.sleep()

        # stop pump
        self.pub2.publish(0)
        r2.sleep()
        r1.sleep()
        pos.x = 120
        pos.y = 0
        pos.z = 35
        self.pub1.publish(pos)
        r1.sleep()
        return 'succeeded'
    # 转动机器人到一定角度       
    def turn_body(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.25
        rate = rospy.Rate(10)
        for i in range(40):            
            self.cmd_vel_pub.publish(cmd_vel)            
            rate.sleep()
       

        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")


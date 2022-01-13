#!/usr/bin/env python
import os
import sys
import time
import cv2
from threading import Thread, Lock, Event
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
from std_msgs.msg import String
from dl_msgs.msg import ObjectInfo
from spark_carry_object.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# receive the object name for detection
OBJECT_NAME = rospy.get_param('/obj_pointto/obj_name')
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_X_CENTER = IMAGE_WIDTH / 2
IMAGE_Y_CENTER = IMAGE_HEIGHT / 2
PATH = os.environ['HOME'] + '/thefile.txt'

class dl_pointto():

    def __init__(self):
        # Subscribe dimensions and information of the detected object
        self.obj_info = ObjectInfo()
        self.cv_brg = CvBridge()

        self.depth_image = None
        self.depth_array = None
        self.object_still = False
        self.point_done = False
        self.turn_done = False
        self.r = 0

        self.obj_info_sub = rospy.Subscriber("objects_info", ObjectInfo, self.obj_info_cb)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image, self.depth_cb, queue_size=1)
        self.arm_pos_pub = rospy.Publisher("position_write_topic", position, queue_size=5)
        self.arm_status_pub = rospy.Publisher('swiftpro_status_topic', status, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.init_everything()
    # Subscribe the object information
    def obj_info_cb(self, obj):
        global xc, yc, xc_prev,yc_prev, find_count

        self.obj_info = obj
        rospy.loginfo(obj.name)

        _, obj_loc, _ = self.get_object_needed_info()
        xc, yc = self.cal_obj_center(obj_loc)
        
        if self.check_object_found():

            if find_count >= 10:
                self.object_still = True
            else:
                if abs(xc-xc_prev) <= 20 and abs(yc-yc_prev) <= 20:
                    find_count += 1
                else:
                    find_count = 0
        else:
            find_count = 0
        # rospy.loginfo(find_count)
        xc_prev = xc
        yc_prev = yc

    def depth_cb(self,obj):
        global xc, yc
        self.depth_image = self.cv_brg.imgmsg_to_cv2(obj, "32FC1")
        self.depth_array = np.array(self.depth_image, dtype=np.float32)

        self.r = self.depth_array[yc,xc]

    # Initialize position, arm pose...
    def init_everything(self):
        global xc,yc, xc_prev, yc_prev, find_count
        find_count = 0
        xc = 0
        yc = 0
        xc_prev = 0
        yc_prev = 0
        self.restart_arm()

    # Get the object information
    # including name, score, location of the detected object at the image
    def get_object_needed_info(self):
        obj_name = self.obj_info.name
        obj_score = self.obj_info.score
        obj_loc = [self.obj_info.x1, self.obj_info.x2, self.obj_info.y1, self.obj_info.y2]
        return obj_name, obj_loc, obj_score

    # calculate the real position in the image
    # xc,yc
    def cal_obj_center(self, obj_loc):

        xc = int(((obj_loc[0] + obj_loc[1]) / 2) * IMAGE_WIDTH)
        yc = int(((obj_loc[2] + obj_loc[3]) / 2) * IMAGE_HEIGHT)
        return xc, yc

    # check if the object input is found
    def check_object_found(self):
        object_found = False
	object_count = 0
	
        if (self.obj_info.name == OBJECT_NAME) and (self.obj_info.score >= 0.5):
            object_found = True
        return object_found

    def execute_(self):
        global find_count
        while not self.object_still:
            rospy.sleep(4)
            #rospy.loginfo("Object not found")
            rospy.loginfo("find_count"+str(find_count))

        while not self.point_done:
            self.start_pointing()

        while not self.turn_done:
            self.start_turning()

        self.restart_arm()

    # exectue turning
    def start_turning(self):
        global xc,yc,find_count

        a, b = self.read_calibrate_mat()
        cmd_vel = Twist()
        t = 1

        _, obj_loc, _ = self.get_object_needed_info()
        xc, yc = self.cal_obj_center(obj_loc)

        s = b[0] * xc + b[1]
        v = s / t

        # 500 mm
        while not self.turn_done:
            if self.r >= 500:
                cmd_vel.angular.z = v/self.r
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("turn rate is: " + str(cmd_vel.angular.z)+ " rad/s")
                rospy.sleep(3)

                rospy.loginfo("Base Turning Done!")
                self.turn_done = True

            elif self.r == 0 or self.r < 500:
                cmd_vel.angular.z = v/400
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("turn rate is: " + str(cmd_vel.angular.z) +" rad/s ")
                rospy.sleep(3)

                rospy.loginfo("Base Turning Done!")
                self.turn_done = True

    # execute pointing
    def start_pointing(self):
        global find_count, xc_prev, yc_prev
        arm_pose = position()
        a, b = self.read_calibrate_mat()

        _, obj_loc, _ = self.get_object_needed_info()
        xc, yc = self.cal_obj_center(obj_loc)
	rospy.loginfo(str(xc))
        while not self.point_done:

            arm_pose.x = 250
            arm_pose.y = int(b[0] * xc + b[1])
            arm_pose.z = 150
            rospy.loginfo("Arm pose is: " + str(arm_pose.y) + " mm")
            self.arm_pos_pub.publish(arm_pose)
            self.arm_status_pub.publish(1)
            rospy.loginfo("Arm pointing done!")
            rospy.sleep(5)
            self.point_done = True

    # set the arm to initial position
    # only y axis will change when pointing to the detected object
    def restart_arm(self):
        rospy.sleep(1)
        arm_pose = position()
        arm_pose.x = 250
        arm_pose.y = 0
        arm_pose.z = 150
        self.arm_pos_pub.publish(arm_pose)
        rospy.loginfo("Arm Reset Done!")
        rospy.sleep(6)

    # read the calibration file
    def read_calibrate_mat(self):
        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        file_pix.close()
        arr = s.split()
        a1 = arr[0]
        a2 = arr[1]
        a3 = arr[2]
        a4 = arr[3]
        a = [0] * 2
        b = [0] * 2
        a[0] = float(a1)  # x-axis coefficient
        a[1] = float(a2)  # x-axis interception
        b[0] = float(a3)  # y-axis coefficient
        b[1] = float(a4)  # y-axis interception

        return a, b

if __name__ == '__main__':
    rospy.init_node('point_object')
    obj = dl_pointto()

    while not rospy.is_shutdown():
        obj.__init__()
        obj.execute_()
        


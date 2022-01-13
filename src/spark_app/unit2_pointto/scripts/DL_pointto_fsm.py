#!/usr/bin/env python
# coding: utf-8

import os
import sys
import time
import cv2
import smach
import math
import smach_ros
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
from dl_msgs.msg import ObjectInfo, DetectionArray
from spark_carry_object.msg import *
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Twist

from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer

# receive the object name for detection
OBJECT_NAME = rospy.get_param('/obj_pointto/obj_name')
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_X_CENTER = IMAGE_WIDTH / 2
IMAGE_Y_CENTER = IMAGE_HEIGHT / 2
PATH = os.environ['HOME'] + '/thefile.txt'

class ObjectDetect(State):
    """
    检测对应的物体
    """
    def __init__(self):
        State.__init__(self,outcomes=['obj_found'],output_keys=['imgloc_out','realloc_out','arm_state_out'])
        self.object_found = False
        self.obj_info = ObjectInfo()
        self.loc = []
        
    def execute(self,userdata):
        self.object_found = False
        self.init_cb()
        
        while not self.object_found:
            rospy.sleep(1)
            print("Object not found...")
            
        self.object_info.unregister()
        self.object_loc.unregister()
        
        global xc
        userdata.imgloc_out = xc
        userdata.realloc_out = self.loc
        userdata.arm_state_out = 0
        rospy.loginfo("%s Found" % OBJECT_NAME)
        
        return 'obj_found'
            
    def init_cb(self):
        global xc,yc,xc_prev,yc_prev,found_count
        xc = 0
        yc = 0
        xc_prev = xc
        yc_prev = yc
        found_count = 0
        self.object_info = rospy.Subscriber("object_info_list", DetectionArray,self.detect_cb)
        self.object_loc = rospy.Subscriber("/camera/depth/points",PointCloud2,self.loc_cb)
        
    def detect_cb(self,objects):
        global xc,yc,xc_prev,yc_prev,found_count
        for obj in objects.objectinfo:
            if obj.name == OBJECT_NAME:
                self.obj_info = obj
                xc = int(((obj.x1 + obj.x2) / 2) * IMAGE_WIDTH)
                yc = int(((obj.y1 + obj.y2) / 2) * IMAGE_HEIGHT)
                if found_count >= 8:
                    self.object_found = True
                else:
                    if abs(xc-xc_prev) <= 15 and abs(yc-yc_prev) <= 15:
                        found_count += 1
                    else:
                        found_count = 0
            else:
                found_count = 0
        xc_prev = xc
        yc_prev = yc
                    
    def loc_cb(self,pointcloud):
        x_range = range(int(self.obj_info.x1*IMAGE_WIDTH),int(self.obj_info.x2*IMAGE_WIDTH))
        y_range = range(int(self.obj_info.y1*IMAGE_HEIGHT),int(self.obj_info.y2*IMAGE_HEIGHT))
        z_temp = 0
        x_temp = 0
        z_list = []
        x_list = []
        x_mean = 0
        z_mean = 0
        for j in y_range:
            smallest = 100000
            for i in x_range:
                pc = point_cloud2.read_points(pointcloud,field_names=("x","y","z"),skip_nans=True,uvs=[[i,j]])
                
                for p in pc:
                    if p[2] < smallest:
                        smallest = p[2]
                        z_temp = p[2]
                        x_temp = -p[0]
            z_list.append(z_temp)
            x_list.append(x_temp)
        
        if z_list or x_list:
            x_mean = sum(x_list)/len(x_list)
            z_mean = sum(z_list)/len(z_list)
            
        self.loc = [x_mean,z_mean]

class MoveArm(State):
    """
    移动机械臂指向物体
    """
    def __init__(self):
        State.__init__(self, outcomes=['point_done','return_done'],input_keys=['obj_arm_in','arm_state_in'])
        self.arm_pub = rospy.Publisher("position_write_topic",position,queue_size=5)
        self.reset_arm()
        
    def execute(self,userdata):
        if userdata.arm_state_in == 0:
            arm_pose = position()
            a,b = self.get_imgmat()
            xc = userdata.obj_arm_in
            arm_pose.x = 250
            arm_pose.y = int(b[0] * xc + b[1])
            arm_pose.z = 150
            rospy.loginfo("Pointing...")
            self.arm_pub.publish(arm_pose)
            rospy.loginfo("Pointing Done!")
            rospy.sleep(5)
            return 'point_done'
        elif userdata.arm_state_in == 1:
            self.reset_arm()
            rospy.loginfo("Reset Done!")
            return 'return_done'
        
    def reset_arm(self):
        rospy.sleep(1)
        arm_pose = position()
        arm_pose.x = 250
        arm_pose.y = 0
        arm_pose.z = 150
        self.arm_pub.publish(arm_pose)
        rospy.loginfo("Arm Reseting...")
        rospy.sleep(6)

    def get_imgmat(self):
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

class MoveBase(State):
    """
    底盘复位到中心点
    """
    def __init__(self):
        State.__init__(self, outcomes=['move_done'],input_keys=['real_loc_in'],output_keys=['arm_state_out'])
        self.base_pub = rospy.Publisher("cmd_vel",Twist,queue_size=2)
        
    def execute(self,userdata):
        loc = userdata.real_loc_in
        cmd = Twist()
        x = loc[0]
        z = loc[1]
        
        if x != 0 and z != 0:
            dist = math.sqrt(x*x + z*z)
            z_angular = math.asin(x/dist)
            cmd.angular.z = z_angular
            rospy.loginfo(cmd)
            self.base_pub.publish(cmd)
            rospy.loginfo("Turning...")
            rospy.sleep(2)
        else:
            self.base_pub.publish(cmd)
            rospy.sleep(2)
            
        userdata.arm_state_out = 1
        rospy.loginfo("Turning Done!")
        return 'move_done'

class PointToObject():
    """
    主程序
    """
    def __init__(self):
        rospy.init_node('obj_pointto', anonymous=False)
       
        rospy.on_shutdown(self.shutdown)
        
        self.pointto_sm = None
        
        self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        
        self.run_sm()
        
        rospy.spin()
        self.pointto_sm.stop()
        
    def run_sm(self):
        self.build_state_machine()
        sis = IntrospectionServer('pointto_sm_server',self.pointto_sm,'/sm_root')
        sis.start()
        
        self.pointto_sm.execute()
        
    def build_state_machine(self):
        if not self.pointto_sm is None:
            self.pointto_sm = None
        self.pointto_sm = StateMachine(outcomes=['succeeded'])
        
        with self.pointto_sm:
            StateMachine.add('ObjectDetect',ObjectDetect(),transitions={'obj_found':'MoveArm'},
                        remapping={'imgloc_out':'img_loc','realloc_out':'real_loc','arm_state_out':'arm_state'})
            StateMachine.add('MoveArm',MoveArm(),transitions={'point_done':'MoveBase','return_done':'ObjectDetect'},
                        remapping={'obj_arm_in':'img_loc','arm_state_in':'arm_state'})
            StateMachine.add('MoveBase',MoveBase(),transitions={'move_done':'MoveArm'},
                        remapping={'real_loc_in':'real_loc','arm_state_out':'arm_state'})
        
    def shutdown(self):
        cmd_vel = position()
        rosy.loginfo("Stopping...")
        self.cmd_pub.publish(cmd_vel)
        rospy.sleep(1)

if __name__=='__main__':
    try:
        PointToObject()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finised Pointing Object")
    
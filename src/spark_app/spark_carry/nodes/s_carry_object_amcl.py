#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
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
from spark_carry_object.srv import *
from spark_carry_object.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from tf import TransformListener
import tf

import actionlib
from actionlib import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Pose, Point, Quaternion

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

class GraspObject(State):
    '''
    抓取物体 功能
    '''    
        
    def __init__(self):
        '''
        初始化
		'''
        #global sub, pub1, pub2
        
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.task = 'grasp object'
        self.is_found_object = False

    def execute(self, userdata):
        '''
        执行状态
        :param userdata:
        '''
        self.is_found_object = False
        self.onInit()
        rate = rospy.Rate(10)

        while not self.is_found_object:
            rate.sleep()
            print("not found\n")
        print("unregisting sub\n")
        self.sub.unregister()
        print("unregisted sub\n")    
        self.grasp()
        return 'succeeded'
    
    def onInit(self):
        global xc, yc, xc_prev, yc_prev, found_count
        xc = 0
        yc = 0
        xc_prev = xc
        yc_prev = yc
        found_count = 0
        #rospy.init_node('image_converter', anonymous=True)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
        self.pub1 = rospy.Publisher('position_write_topic', position, queue_size=10)
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        
    
    def grasp(self):
        print("start to grasp\n")
        global xc, yc, found_count
        # stop function


	file_pix = open('/home/zlt/spark_ws/thefile.txt', 'r')
	s = file_pix.read()
	file_pix.close()
	print(s);
     	arr=s.split()
     	a1=arr[0]
	a2=arr[1]
	a3=arr[2]
	a4=arr[3]
     	#a4=arr[3].replace('\n','') #readline 读取文件的时候，默认加上“\n"
     	#(a1)
     	#print(a2)
	#print(a3)
	#print(a4)
     	#n1=float(a1)*2
	a = [0]*2
	b = [0]*2
     	a[0]=float(a1)
        a[1]=float(a2)
     	b[0]=float(a3)
        b[1]=float(a4)
	print('k and b value:',a[0],a[1],b[0],b[1])

        r1 = rospy.Rate(0.05)
        r2 = rospy.Rate(10)
        # a = [0.1791, -0.3121, -0.0013, -0.0002, -0.0021, 256.1421]
        # b = [-1.0607, -0.4061, 0.0009, 0.0002, 0.0012, 325.9349]
	# a = [-0.3180, -0.8592, 0.0023, 0.0009, 0.0007, 225]
	# b = [0.6361, 1.7184, -0.0046, -0.0017, -0.0014, -8.2997]
        #a = [-0.9193, 362.29]
	#b = [-0.9339, 307.38]
        # a = [-0.8213, 297.6527]
	# b = [-0.8919, 311.0063]
	pos = position()
        # start pump
        self.pub2.publish(1)
        r2.sleep()
        # go forward
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
        # go back home
        pos.x = 250  #160
        pos.y = 0
        pos.z = 115  #55
        self.pub1.publish(pos)
        r1.sleep()
        
    def image_cb(self, data):
        #print("received image\n")
        global xc, yc, xc_prev, yc_prev, found_count
        # change to opencv
        try:
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('error')

        # change rgb to hsv
        cv_image2 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)

        # extract blue
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
        #cv2.imshow("win1", cv_image1)
        #cv2.imshow("win2", cv_image5)
        #cv2.waitKey(1)
        contours, hier = cv2.findContours(cv_image5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # if find contours, pick the biggest box
        if len(contours) > 0:
            size = []
            size_max = 0
            for i, c in enumerate(contours):
                rect = cv2.minAreaRect(c)
                box = cv2.cv.BoxPoints(rect)
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
            #print found_count
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
 

class ReleaseObject(State):
    '''
    释放物体 功能
    '''
    def __init__(self):
        '''
        初始化
        '''
        global pub1, pub2
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.task = 'release object' 
        #rospy.init_node('image_converter', anonymous=True)
        pub1 = rospy.Publisher('position_write_topic', position, queue_size=10)
        pub2 = rospy.Publisher('pump_topic', status, queue_size=4)   
        
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        global pub1, pub2
        r1 = rospy.Rate(0.15)  # 5s
        r2 = rospy.Rate(1)     # 1s
        pos = position()
        # go forward
        pos.x = 200
        pos.y = 0
        pos.z = -40  #-80
        pub1.publish(pos)
        r1.sleep()

        # stop pump
        pub2.publish(0)
        r2.sleep()
	r1.sleep()
        # go back home
        pos.x = 120
        pos.y = 0
        pos.z = 35
        pub1.publish(pos)
        r1.sleep()
        return 'succeeded'
            

# 获得厨房位置  
class GetLocation(State):
    '''
    生成导航位置
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['waypoint_out'])     
        self.nav_pose = [rospy.get_param("~a_Pose"), rospy.get_param("~b_Pose")]
        self.pose_idx = 0      
        
    def execute(self, userdata):
        '''
        状态执行函数
        :param userdata:
        '''
        self.loc_pose = self.get_pose()
        if self.loc_pose is None:
            return "aborted"
        userdata.waypoint_out = self.loc_pose
        return 'succeeded' 
    
    def get_pose(self):
        '''
        获得导航点的位置和姿态
        '''
        self.pose_idx = (self.pose_idx + 1) % 2
        # to generate pose
        pose = self.split_str(self.nav_pose[self.pose_idx])
        print pose                  
        return pose
    
    def split_str(self, str):
        
        loc_array = str.split(',')
        if len(loc_array) != 7:
            return None
        pose = Pose(Point(0, 0, 0), Quaternion(0.0, 0.0, 0.0, 1.0))
        pose.position.x = float(loc_array[0])
        pose.position.y = float(loc_array[1])
        pose.position.z = float(loc_array[2])
        pose.orientation.x = float(loc_array[3])
        pose.orientation.y = float(loc_array[4])
        pose.orientation.z = float(loc_array[5])
        pose.orientation.w = float(loc_array[6])
        
        return pose
    
class Nav2Waypoint(State):
    '''
    导航：巡航到指定的地点，通过move base
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded', 'aborted'],
                       input_keys=['waypoint_in'])
        
        # Subscribe to the move_base action server        
        # self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)        
        # Wait up to 60 seconds for the action server to become available
        global move_base;
        move_base.wait_for_server(rospy.Duration(5))    
        
        rospy.loginfo("Connected to move_base action server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "/map"

    def execute(self, userdata):
        '''
        状态执行函数
        :param userdata:用户数据
        '''
        self.goal.target_pose.pose = userdata.waypoint_in
        
        #return 'succeeded'
        
        self.goal.target_pose.header.stamp = rospy.Time.now() 
        if self.goal.target_pose.pose is None:
            return 'aborted'
        # Send the goal pose to the MoveBaseAction server
        global move_base
        move_base.send_goal(self.goal)
        
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('wait for result')
        
        # Allow 1 minute to get there
        finished_within_time = move_base.wait_for_result(rospy.Duration(60)) 

        # If the robot don't get there in time, abort the goal
        if not finished_within_time:
            move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'aborted'
        else:
            # We made it!
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            return 'succeeded'


class TurnBody(State):
    '''
    转动身体 功能
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.task = 'turn body'        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  
        self.min_z = rospy.get_param("~turnbody_min_z", 0.2)
        self.max_z = rospy.get_param("~turnbody_max_z", 0.6)
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        cmd_vel = Twist()
        cmd_vel.angular.z = random.uniform(self.min_z, self.max_z)
        print cmd_vel.angular.z
        rate = rospy.Rate(25)
        for i in range(25):            
            self.cmd_vel_pub.publish(cmd_vel)            
            rate.sleep()
            
        # self.cmd_vel_pub.publish(cmd)
        return 'succeeded'    

class Go2Goal(State):
    '''
    转动身体 功能
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.task = 'go line'        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  
        self.min_z = rospy.get_param("~turnbody_min_z", 0.2)
        self.max_z = rospy.get_param("~turnbody_max_z", 0.6)
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        cmd_vel = Twist()
        #cmd_vel.angular.z = random.uniform(self.min_z, self.max_z)
	cmd_vel.linear.x = 0.15
        print cmd_vel.angular.z
        rate = rospy.Rate(10)
        for i in range(60):            
            self.cmd_vel_pub.publish(cmd_vel)            
            rate.sleep()
            
        # self.cmd_vel_pub.publish(cmd)
        return 'succeeded'    


class TurnBodyQuar(State):
    '''
    转动身体 功能
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.task = 'turn body quar'        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  
        self.min_z = rospy.get_param("~turnbody_min_z", 0.2)
        self.max_z = rospy.get_param("~turnbody_max_z", 0.6)
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.25
        print cmd_vel.angular.z
        rate = rospy.Rate(10)
        for i in range(20):            
            self.cmd_vel_pub.publish(cmd_vel)            
            rate.sleep()
            
        # self.cmd_vel_pub.publish(cmd)
        return 'succeeded'    
    
class EndAbort(State):
    '''
    提示结果失败
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded'])
        # self.play_sound  = PlaySound()
        self.task = 'end abort'        
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        return 'succeeded'        

class EndSuccess(State):
    '''
    提示结果成功
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded'])
        # self.play_sound  = PlaySound()
        self.task = 'end success'        
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        self.sayword_pub.publish("任务完成");
        return 'succeeded'
        
                   
                
class CarryObject():
    '''
    场景：搬运物体
    功能：将目标运行到指定的位置
    '''
    def __init__(self):
        '''
        初始化
        '''
        rospy.init_node('s_carry_object_node', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        self.switch_status = 0;
        self.sm_carry_object = None
        self.thr = None
        self.init_service_and_topic()
        # 控制
        self.stopworld = "停止处理"
        self.loaction_name = None
          
        
    def scene_status_swith_cb(self, req):
        '''
        场景状态切换回调函数，用于启动场景或者关闭场景
        :param req:请求参数
        '''
        print "-----------req status:%d" % req.END
        return_flag = -1
        if req.type == req.END:
             self.switch_status = 0
             global move_base
             move_base.cancel_goal()                
             for i in range(10):
                 self.cmd_vel_pub.publish(Twist())
                 
             if not self.thr is None:
                 try:
                     self.terminate_thread(self.thr)
                 except e:
                     pass  
                      
             return_flag = 0       
        if req.type == req.RUN:
             if self.switch_status == 0:
                 # thread.start_new_thread(self.timer, (1,1)) 
                 self.thr = threading.Thread(target=self.timer, args=(1, 1))
                 self.thr.start()
                 self.switch_status = 1
             return_flag = 1
         
        # self.master_s_pub.publish("",0)
        print "s_carry_object status:%d" % return_flag
        return sceneResponse(return_flag) 
    
    def timer(self, no, interval):
        '''
        计时器
        :param no:
        :param interval:计时器间隔
        '''
        # time.sleep(interval);
        rospy.loginfo("req infomaion, status:%s" % (self.switch_status))
        if self.switch_status == 1:
            rospy.loginfo("starting to sm")
            self.build_state_machine();
            self.execute_sm()
        
        self.switch_status = 0
        rospy.loginfo("stopped to sm")
        thread.exit_thread() 


    def terminate_thread(self, thread):
        """
        Terminates a python thread from another thread.
        中止结束线程
        :param thread: a threading.Thread instance
        """
        try:
            if not thread.isAlive():
                return
    
            exc = ctypes.py_object(SystemExit)
            res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
                ctypes.c_long(thread.ident), exc)
            if res == 0:
                raise ValueError("nonexistent thread id")
            elif res > 1:
                # """if it returns a number greater than one, you're in trouble,
                # and you should call it again with exc=NULL to revert the effect"""
                ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
                raise SystemError("PyThreadState_SetAsyncExc failed")
        except e:
            pass
        self.cmd_vel_pub.publish(Twist())
    
    def init_service_and_topic(self):
        '''
        初始化服务和话题
        '''
        self.switch_status_service = rospy.Service('/s_carry_object', scene, self.scene_status_swith_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  
         
    def build_state_machine(self): 
        '''
        建立状态机
        '''
        if not self.sm_carry_object is None:
            self.sm_carry_object = None
        self.sm_carry_object = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with self.sm_carry_object:
            StateMachine.add('GRASP_OBJECT', GraspObject(), transitions={'succeeded':'GET_LOCATION',
                                                                               'aborted':'END_ABORT'})
            StateMachine.add('GET_LOCATION', GetLocation(), transitions={'succeeded':'NAV_TO_WAYPOINT',
                                                                               'aborted':'END_ABORT'},
                             remapping={'waypoint_out':'nav_waypoint'})
            #StateMachine.add('NAV_TO_WAYPOINT', Nav2Waypoint(), transitions={'succeeded':'RELEASE_OBJECT','aborted':'END_ABORT'},

            StateMachine.add('NAV_TO_WAYPOINT', Go2Goal(), transitions={'succeeded':'RELEASE_OBJECT',
                                                                               'aborted':'END_ABORT'},
                             remapping={'waypoint_in':'nav_waypoint'}) 
            StateMachine.add('RELEASE_OBJECT', ReleaseObject(), transitions={'succeeded':'TURN_BODY',
                                                                               'aborted':'END_ABORT'})
            #StateMachine.add('TURN_BODY', TurnBody(), transitions={'succeeded':'GRASP_OBJECT','aborted':'END_ABORT'})
	    StateMachine.add('TURN_BODY', TurnBody(), transitions={'succeeded':'GRASP_OBJECT','aborted':'END_ABORT'})
            StateMachine.add('END_ABORT', EndAbort(), transitions={'succeeded':'succeeded'}) 
            StateMachine.add('END_SUCESS', EndSuccess(), transitions={'succeeded':'succeeded'}) 
    def stop_cb(self, msg):
        '''
        停止回调函数
        :param msg:
        '''
        # rospy.loginfo("停止指令："+msg.data)
        print msg.data
        if msg.data == self.stopworld:
            switch_client = rospy.ServiceProxy('s_carry_object', scene)
            type = 0
            param = ""
            respon = switch_client(type, param)

            
    def execute_sm(self):   
        '''
        运行状态即机
        '''
        intro_server = IntrospectionServer('sm_carry_object', self.sm_carry_object, '/SM_ROOT')
        intro_server.start() 
        # Execute the state machine
        sm_outcome = self.sm_carry_object.execute()        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
        intro_server.stop()
          
    def shutdown(self):
        '''
        关闭状态机
        '''
        rospy.loginfo("Stopping carry object")            
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())
        # rospy.spin()

if __name__ == '__main__':
    try:
        rospy.loginfo("Init carry object")   

	file_pix = open('/home/zlt/spark_ws/thefile.txt', 'r')
	s = file_pix.read()
	file_pix.close()
	print(s);
     	arr=s.split()
     	a1=arr[0]
	a2=arr[1]
	a3=arr[2]
	a4=arr[3]
     	#a4=arr[3].replace('\n','') #readline 读取文件的时候，默认加上“\n"
     	print(a1)
     	print(a2)
	print(a3)
	print(a4)
     	n1=float(a1)*2
	a = [0]*2
	b = [0]*2
     	a[0]=float(a1)
        a[1]=float(a2)
     	b[0]=float(a3)
        b[1]=float(a4)
	print('k and b value:',a[0],a[1],b[0],b[1])

        CarryObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finised  carry object")


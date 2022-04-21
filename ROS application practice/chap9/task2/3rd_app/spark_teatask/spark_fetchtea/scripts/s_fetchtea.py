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
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from spark_fetchtea.srv import *
# from cv_bridge import CvBridge, CvBridgeError

from tf import TransformListener
import tf

import actionlib
from actionlib import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Pose, Point, Quaternion




'''
import sys, re,os
libpath=os.environ['HOME']+"/nxrobo_package/devel/lib/"
sys.path.append(libpath)
import libnx_voice
reload(sys)
sys.setdefaultencoding("utf-8")
'''

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
global location_name
location_name = ""
loc_name2playword = {'goal':'好的，去端茶','home':'好的，回家'}
loc_name2rosparam = {'goal':'/kitchen_pose','home':'/robot_pose'}
loc_name2playword_ontheway = {'goal':'太阳当空照，花儿对我笑，小鸟说早早早，为何背上小书包','home':'回家咯，啦啦，啦啦'}
        
class SpeaktoTipStart(State):
    '''
    提示开始端茶指令 功能
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        # self.play_sound  = PlaySound()
        self.task = 'play2tip'        
        self.sayword_pub = rospy.Publisher("/voice/tts", String, queue_size=1)
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        global location_name
        if not loc_name2playword.has_key(location_name):
            rospy.loginfo('no the given goal')
            return 'aborted'
        self.sayword_pub.publish(loc_name2playword[location_name]);
        return 'succeeded'        

# 获得厨房位置  
class GetLocation(State):
    '''
    获得厨房位置
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['waypoint_out'])     
        self.loc_pose = None;    
        
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
        pose = None
        global location_name
        if not loc_name2rosparam.has_key(location_name):
            return pose
        pose = rospy.get_param(loc_name2rosparam[location_name]);
        print pose        
        
        return self.split_str(pose)
    
    def split_str(self, str):
        
        loc_array = str.split(',')
        if len(loc_array) != 7:
            return None
        pose = Pose(Point(0,0,0), Quaternion(0.0, 0.0, 0.0, 1.0))
        
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
    
        self.sayword_pub = rospy.Publisher("/voice/tts", String, queue_size=1)

    def execute(self, userdata):
        '''
        状态执行函数
        :param userdata:用户数据
        '''
        self.goal.target_pose.pose = userdata.waypoint_in
        
        
        
        self.goal.target_pose.header.stamp = rospy.Time.now() 
        if self.goal.target_pose.pose is None:
            return 'aborted'
        # Send the goal pose to the MoveBaseAction server
        global move_base
        move_base.send_goal(self.goal)
        
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('wait for result..')
        
        global location_name
	self.sayword_pub.publish(loc_name2playword_ontheway[location_name]);
        
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


class EndSpeaktoTipAbort(State):
    '''
    提示语音提示结果失败
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded'])
        # self.play_sound  = PlaySound()
        self.task = 'play2tip'        
        self.sayword_pub = rospy.Publisher("/voice/tts", String, queue_size=1)
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        self.sayword_pub.publish("迷路了，任务失败");
        return 'succeeded'        

class EndSpeaktoTipSucess(State):
    '''
    提示语音提示结果成功
    '''
    def __init__(self):
        '''
        初始化
        '''
        State.__init__(self, outcomes=['succeeded'])
        # self.play_sound  = PlaySound()
        self.task = 'play2tip'        
        self.sayword_pub = rospy.Publisher("/voice/tts", String, queue_size=1)
    def execute(self, userdata):        
        '''
        执行状态
        :param userdata:
        '''
        self.sayword_pub.publish("任务完成");
        return 'succeeded'
        
                   
                
class FetchTea():
    '''
    场景：到指定位置
    功能：将目标运行到指定的位置
    '''
    def __init__(self):
        '''
        初始化
        '''
        rospy.init_node('s_fetchtea_node', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        self.switch_status = 0;
        self.sm_fetchtea = None
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
                 global location_name
                 location_name = req.param
                 print location_name
                 # thread.start_new_thread(self.timer, (1,1)) 
                 self.thr = threading.Thread(target=self.timer, args=(1, 1))
                 self.thr.start()
                 self.switch_status = 1
             return_flag = 1
         
        # self.master_s_pub.publish("",0)
        rospy.loginfo("req infomaion, type:%s,param:%s" % (req.type, req.param))
        print "s_fetchtea status:%d" % return_flag
        return sceneResponse(return_flag) 
    
    def timer(self, no, interval):
        '''
        计时器
        :param no:
        :param interval:计时器间隔
        '''
        #time.sleep(interval);
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
        self.switch_status_service = rospy.Service('/s_fetchtea', scene, self.scene_status_swith_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  
         
    def build_state_machine(self): 
        '''
        建立状态机
        '''
        if not self.sm_fetchtea is None:
            self.sm_fetchtea = None
        self.sm_fetchtea = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with self.sm_fetchtea:
            StateMachine.add('START_TIP', SpeaktoTipStart(), transitions={'succeeded':'GET_LOCATION',
                                                                               'aborted':'END_SPEAK_TO_TIP_ABORT'})
            StateMachine.add('GET_LOCATION', GetLocation(), transitions={'succeeded':'NAV_TO_WAYPOINT',
                                                                               'aborted':'END_SPEAK_TO_TIP_ABORT'},
                             remapping={'waypoint_out':'nav_waypoint'})
            StateMachine.add('NAV_TO_WAYPOINT', Nav2Waypoint(), transitions={'succeeded':'END_SPEAK_TO_TIP_SUCESS',
                                                                               'aborted':'END_SPEAK_TO_TIP_ABORT'},
                             remapping={'waypoint_in':'nav_waypoint'}) 
            StateMachine.add('END_SPEAK_TO_TIP_ABORT', EndSpeaktoTipAbort(), transitions={'succeeded':'succeeded'}) 
            StateMachine.add('END_SPEAK_TO_TIP_SUCESS', EndSpeaktoTipSucess(), transitions={'succeeded':'succeeded'}) 
    def stop_cb(self, msg):
        '''
        停止回调函数
        :param msg:
        '''
        # rospy.loginfo("停止指令："+msg.data)
        print msg.data
        if msg.data == self.stopworld:
            switch_client = rospy.ServiceProxy('s_fetchtea', scene)
            type = 0
            param = ""
            respon = switch_client(type, param)

            
    def execute_sm(self):   
        '''
        运行状态即机
        '''
        intro_server = IntrospectionServer('sm_fetchtea', self.sm_fetchtea, '/SM_ROOT')
        intro_server.start() 
        # Execute the state machine
        sm_outcome = self.sm_fetchtea.execute()        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
        intro_server.stop()
          
    def shutdown(self):
        '''
        关闭状态机
        '''
        rospy.loginfo("Stopping Go to Location")            
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())
        # rospy.spin()

if __name__ == '__main__':
    try:
        rospy.loginfo("Init Fetch Tea")   
        FetchTea()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finised  Fetch Tea")


#!/usr/bin/env python
# -*- coding: utf-8 -*
 
import  os
import  sys
import  tty, termios
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
 
# 全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)

global can_grasp
global can_release



def grasp_status_cp(msg):
    global can_release,can_grasp
    # 物体抓取成功,让机器人回起始点
    if msg.data=='1':
        can_release=True
    if msg.data=='0' or msg.data=='-1':
        can_grasp=True
grasp_status=rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=1)

def keyboardLoop():
    #初始化
    rospy.init_node('teleop')
    rate = rospy.Rate(rospy.get_param('~hz', 10))
 
    #速度变量
    walk_vel_ = rospy.get_param('walk_vel', 0.3)
    run_vel_ = rospy.get_param('run_vel', 1.0)
    yaw_rate_ = rospy.get_param('yaw_rate', 1.0)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 1.5)
 
    max_tv = walk_vel_
    max_rv = yaw_rate_
    speed=0
    global can_release,can_grasp
    can_grasp=True
    can_release=False
    
    print "使用[WASD]控制机器人"
    print "按[G]抓取物体，按[H]放下物体"
    print "按[Q]退出"
 
    #读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        turn =0
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
 
        if ch == 'g':
            if can_grasp:
                msg=String()
                msg.data='1'
                grasp_pub.publish(msg)
                can_grasp=False
        elif ch == 'h':
            if can_release:
                msg=String()
                msg.data='0'
                grasp_pub.publish(msg)
                can_release=False
        elif ch == 'w':
            max_tv = walk_vel_
            speed = 1
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -1
            turn = 0
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            turn = 1
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            turn = -1
        elif ch == 'W':
            max_tv = run_vel_
            speed = 1
            turn = 0
        elif ch == 'S':
            max_tv = run_vel_
            speed = -1
            turn = 0
        elif ch == 'A':
            max_rv = yaw_rate_run_
            speed = 0
            turn = 1
        elif ch == 'D':
            max_rv = yaw_rate_run_
            speed = 0
            turn = -1
        elif ch == 'q':
            exit()
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            turn = 0
 
        #发送消息
        cmd.linear.x = speed * max_tv
        cmd.angular.z = turn * max_rv
        pub.publish(cmd)
        rate.sleep()
		#停止机器人
        stop_robot()
 
def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)
 
if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass


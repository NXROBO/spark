#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spark_turn_around_node"); //ROS节点初始化
  ROS_INFO("The spark are turning around!!!");  //终端输出文字
  ros::NodeHandle nh;                        //创建节点句柄
  ros::Publisher cmd_pub;                     //创建一个Publisher

  //发布名为/cmd_vel的topic，消息类型为geometry_msgs::Twist
  cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate rate(10);                          //设置循环的频率
  double prev_sec = ros::Time().now().toSec();     //获取当前时间
  int sec = 12;                                //运动时间12s
  while (ros::ok())
  {
    if (ros::Time().now().toSec() - prev_sec > sec)  //时间是否到了
      break;
    geometry_msgs::Twist vel;                 //速度指令
    vel.linear.x = 0;                        //沿x轴运动的速度
    vel.angular.z = 0.6;                        //绕z轴旋转的速度
    cmd_pub.publish(vel);                    //发布速度话题
    rate.sleep();                            //按照循环频率延时
  }
  ROS_INFO("Spark stop moving!!!");
  ros::spin();                               //循环等待回调函数
  return 0;
}


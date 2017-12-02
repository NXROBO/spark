#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>       // cmd_vel
#include <nav_msgs/Odometry.h>         // odom
#include <tf/transform_broadcaster.h>  // odom_trans
#include <tf/tf.h>
#include <fake_spark_base/fake_spark_base.h>

using namespace nxspark;

SparkStateSimulation::SparkStateSimulation()
{
  double Odom_x = 0;
  double Odom_y = 0;
  double Odom_theta = 0;
}

SparkStateSimulation::~SparkStateSimulation()
{
  cmd_vel_sub.shutdown();
}

bool SparkStateSimulation::onInit(ros::NodeHandle nh)
{
  this->nh = nh;

  return true;
}

int SparkStateSimulation::runEverything()
{
  // cmd_vel subscriber
  cmd_vel_sub =
      nh.subscribe("/cmd_vel", 1, &SparkStateSimulation::commandVelocityCallback, this);  // need to have class name
  prev_t = 0;
  curr_t = 0;
  return 0;
}

// cmd_vel tester
void SparkStateSimulation::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
  ROS_INFO("Received linear_x_cmd_vel: [%f]", linear_x = cmd_vel->linear.x);
  ROS_INFO("Received angular_z_cmd_vel: [%f]", angular_z = cmd_vel->angular.z);
  linear_y = cmd_vel->linear.y;
  linear_z = cmd_vel->linear.z;
  angular_x = cmd_vel->angular.x;
  angular_y = cmd_vel->angular.y;
  angular_z = cmd_vel->angular.z;
  curr_t = ros::Time().now().toSec();
  ROS_INFO("current time is : %f", curr_t);

  if (curr_t - prev_t > 1)
  {
    prev_t = curr_t;
    return;
  }
  pubOdom();
  prev_t = curr_t;

  return;
}

void SparkStateSimulation::pubOdom()
{
  double d_t = 0;
  double delta_x, delta_y, delta_w;

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);

  d_t = curr_t - prev_t;

  delta_x = linear_x * cos(Odom_theta) * d_t;
  delta_y = linear_x * sin(Odom_theta) * d_t;
  delta_w = angular_z * d_t;

  ROS_INFO("delta_x is now: [%f] ", delta_x);
  ROS_INFO("delta_y is now: [%f] ", delta_y);
  ROS_INFO("delta_w is now: [%f]", delta_w);

  Odom_x += delta_x;
  Odom_y += delta_y;
  Odom_theta += delta_w;

  ROS_INFO("The Odom_x: [%f] ", Odom_x);
  ROS_INFO("The Odom_y: [%f] ", Odom_y);
  ROS_INFO("The Odom_theta: [%f]", Odom_theta);

  // publish the transforms over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom = "odom";
  base_footprint = "base_footprint";
  odom_trans.header.frame_id = odom.c_str();
  odom_trans.child_frame_id = base_footprint.c_str();
  odom_trans.transform.translation.x = Odom_x;
  odom_trans.transform.translation.y = Odom_y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(Odom_theta);
  tf_broadcaster.sendTransform(odom_trans);  // publish all odom_trans
  ROS_INFO("test odom_transform");

  // publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom2 = "odom";
  base_footprint2 = "base_footprint";
  odom.header.frame_id = odom2.c_str();
  odom.pose.pose.position.x = Odom_x;
  odom.pose.pose.position.y = Odom_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(Odom_theta);
  odom.child_frame_id = base_footprint2.c_str();
  odom.twist.twist.linear.x = linear_x;
  odom.twist.twist.linear.y = linear_y;
  odom.twist.twist.angular.z = angular_z;

  odom_pub.publish(odom);  // publish all odom
  ROS_INFO("test odom");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spark_fake_base");
  ros::NodeHandle nh;
  nxspark::SparkStateSimulation ss_sim;  // blueprint + car
  ss_sim.onInit(nh);                     // car.drive(initialize by passing in nh)
  ss_sim.runEverything();                // master function call
  ros::spin();

  return 0;
}

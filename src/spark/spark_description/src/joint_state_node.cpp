/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  Copyright (c) 2016, NXROBO Ltd.
 *  Xiankai Chen <xiankai.cheng@nxrobo.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

//#define M_PI 3.14159265359
#define PITHCH_MAX (M_PI)
#define PITHCH_MIN (-M_PI)
#define ROLL_MAX (M_PI)
#define ROLL_MIN (-M_PI)

ros::Publisher joint_pub;
ros::Subscriber servo_motor_state_sub;

void servo_motor_cb()
{
  double left_wheel = 0;
  double right_wheel = 0;
  ros::Rate r(10);
  while (1)
  {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] = "left_wheel_joint";
    joint_state.position[0] = right_wheel;
    joint_state.name[1] = "right_wheel_joint";
    joint_state.position[1] = left_wheel;

    joint_pub.publish(joint_state);
    r.sleep();

    right_wheel += 0.1;
    left_wheel += 0.1;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_node");
  ros::NodeHandle n;
  joint_pub = n.advertise<sensor_msgs::JointState>("wheel_states", 1);
  servo_motor_cb();

  ros::spin();

  return 0;
}

/*
  * Copyright (c) 2016, SHENZHEN NXROBO Co.,LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Xiankai Chen (xiankai.chen@nxrobo.com) and Jian Song
 *		(jian.song@nxrobo.com)
 */
#ifndef SPARK_APP_SPARK_FOLLOWER_SRC_NXFOLLOWER_HPP_
#define SPARK_APP_SPARK_FOLLOWER_SRC_NXFOLLOWER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>

namespace nxfollower
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class NxFollowerNode
{
private:
  ros::NodeHandle nhandle;
  ros::NodeHandle pnhandle;
  // cmd_vel publisher
  ros::Publisher cmdvel_pub;
  // point clound subscriber
  ros::Subscriber cloud_sub;

  double min_y_;   /**< The minimum y position of the points in the box. */
  double max_y_;   /**< The maximum y position of the points in the box. */
  double min_x_;   /**< The minimum x position of the points in the box. */
  double max_x_;   /**< The maximum x position of the points in the box. */
  double max_z_;   /**< The maximum z position of the points in the box. */
  double goal_z_;  /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  double z_thre;
  double x_thre;
  double max_vx;                 /*max velocity x*/
  double max_vz;                 /*max velocity z*/
  double max_depth_, min_depth_; /**< The maximum z position of the points in the box. */
  double goal_depth_;            /**< The distance away from the robot to hold the centroid */
  double depth_thre;
  double y_thre;

public:
  NxFollowerNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : min_y_(0.1), max_y_(0.5), min_x_(-0.2), max_x_(0.2), max_z_(0.8), goal_z_(0.6), z_scale_(1.0), x_scale_(5.0)
  {
    nhandle = nh;
    pnhandle = pnh;

    min_x_ = -0.2;
    max_x_ = 0.2;
    min_y_ = -0.1;
    max_y_ = 0.3;
    max_z_ = 1.5;
    goal_z_ = 0.7;
    z_scale_ = 0.8;
    x_scale_ = 2;
    z_thre = 0.05;
    x_thre = 0.05;
    y_thre = 0.087222222;

    max_vx = 0.4;
    max_vz = 0.8;

    max_depth_ = 2;
    min_depth_ = 0.4;
    goal_depth_ = 0.9;
    depth_thre = 0.1;
    y_thre = 0.087222222;

    // public the cmd_vel message
    cmdvel_pub = nhandle.advertise<geometry_msgs::Twist>("/raw_cmd_vel", 1);
    // subscribe the point clound
    cloud_sub = nhandle.subscribe<PointCloud>("/camera/depth/points", 1, &NxFollowerNode::pointCloudCb, this);
  }

  virtual ~NxFollowerNode()
  {
  }

  void pointCloudCb(const PointCloud::ConstPtr &cloud)
  {
    // X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    // Number of points observed
    unsigned int n = 0;
    pcl::PointXYZ pt;
    for (int kk = 0; kk < cloud->points.size(); kk++)
    {
      pt = cloud->points[kk];
      if ((!std::isnan(x) && !std::isnan(y) && !std::isnan(z))&&(!std::isinf(pt.x) && !std::isinf(pt.y) && !std::isinf(pt.z)))
      {
        if (pt.y > min_y_ && pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
        {
          // Add the point to the totals
          x += pt.x;
          z += pt.z;
          n++;
        }
      }
    }

    if (n > 2000)
    {
      x /= n;
      z /= n;
      if (z > max_z_)
      {
        cmdvel_pub.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        return;
      }

      pubCmd(-x, z);
    }
    else
    {
      cmdvel_pub.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
    }
  }

  void pubCmd(const float &y, const float &depth)
  {
    double curr_dist = sqrt(y * y + depth * depth);
    if (curr_dist == 0)
    {
      cmdvel_pub.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      return;
    }

    float x_linear = 0;
    float z_angular = 0;
    float z_scale = 1.2;
    float x_scale = 5.0;	//2.0
    x_linear = (depth - goal_depth_) * z_scale;
    z_angular = asin(y / curr_dist) * x_scale;

    if (depth_thre > fabs(depth - goal_depth_))
      x_linear = 0;
    if (y_thre > y && y > -y_thre)
      z_angular = 0;

    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    cmd->linear.x = x_linear;
    cmd->angular.z = z_angular;

    cmdvel_pub.publish(cmd);
  }

  void spin()
  {
    ros::spin();
  }
};
}

/*main*/
/*int main(int argc, char **argv)
{
  ros::init(argc, argv, "s_spark_follower_node");
  ros::NodeHandle n;

  NxFollowerNode nxfollower(n);
  nxfollower.spin();

  return 0;
}*/

#endif

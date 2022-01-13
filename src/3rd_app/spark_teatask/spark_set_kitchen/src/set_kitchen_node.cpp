/*
 * Big-i is the best robot for family!
 * Copyright (c) 2016.11 ShenZhen NXROBO
 * All rights reserved.
 * Author: mantou
 */
#include <ros/ros.h>
#include <sstream>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

namespace spark_set_kitchen{

/**
 * @brief The SetKitchen class 1. 从rviz中获取点击点的二维坐标
 *                             2. 将clicded_points点加上姿态信息转化为厨房位姿信息发布出去
 *                             3. 在点击点处显示kitchen字符
 *                             4. 转换为目标点发布出去
 */
class SetKitchen
{

public:

  /**
   * @brief SetKitchen 构造函数
   * @param nh
   * @param pnh
   */
  SetKitchen(ros::NodeHandle nh, ros::NodeHandle pnh) :
    nh_(nh),
    pnh_(pnh),
    tf_(ros::Duration(5))
  {
    clicked_point_sut_ = nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &SetKitchen::pointCb, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("kitchen_str_marker", 1);
  }

  /**
   * @brief pointCb 接收点的回调
   * @param clicked_point
   */
  void pointCb(const geometry_msgs::PointStampedConstPtr &clicked_point)
  {
    ROS_INFO("received the point: x = %.2f, y = %.2f", clicked_point->point.x, clicked_point->point.y);
    //发布可视化信息
    pubMarker(clicked_point);
    //获取机器人位姿
    tf::StampedTransform robot_pose_tf;
    try
    {
      tf_.waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(5));
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), robot_pose_tf);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR_STREAM("Couldn't transform from "<<"map"<<" to "<< "odom");
    }
    //向参数服务器写入机器人位姿
    char tmp_robot[100];
    sprintf(tmp_robot, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", robot_pose_tf.getOrigin().x(),robot_pose_tf.getOrigin().y(),robot_pose_tf.getOrigin().z(),
            robot_pose_tf.getRotation().x(),robot_pose_tf.getRotation().y(),robot_pose_tf.getRotation().z(),robot_pose_tf.getRotation().w());
    std::string s_robot(tmp_robot);
    ros::param::set("/robot_pose", s_robot);
    //向参数服务器写入厨房位姿
    char tmp_kitchen[100];
    sprintf(tmp_kitchen, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", clicked_point->point.x,clicked_point->point.y,0.0,
            robot_pose_tf.getRotation().x(),robot_pose_tf.getRotation().y(),robot_pose_tf.getRotation().z(),robot_pose_tf.getRotation().w());
    std::string s_kitchen(tmp_kitchen);
    ros::param::set("/kitchen_pose", s_kitchen);
  }

  /**
   * @brief pubCubeAndStr 发布可视化信息
   */
  void pubMarker(const geometry_msgs::PointStampedConstPtr &clicked_point)
  {
    visualization_msgs::Marker point_marker, text_marker;
    //发布红色点
    point_marker.header.frame_id = "/map";
    point_marker.header.stamp = ros::Time::now();
    point_marker.ns = "points";
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.orientation.w = 1.0;
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::POINTS;
    point_marker.scale.x = 0.2;
    point_marker.scale.y = 0.2;
    point_marker.color.r = 0.0f;
    point_marker.color.g = 1.0f;
    point_marker.color.b = 0.0f;
    point_marker.color.a = 0.5;
    point_marker.points.push_back(clicked_point->point);
    marker_pub_.publish(point_marker);
    //发布kitchen
    text_marker.header.frame_id = "/map";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "points";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.orientation.w = 1.0;
    text_marker.id = 1;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.5;
    text_marker.color.g = 1.0;
    text_marker.color.a = 1.0;
    text_marker.pose.position.x = clicked_point->point.x;
    text_marker.pose.position.y = clicked_point->point.y;
    text_marker.pose.position.z = 0;
    std::string str("kitchen");
    text_marker.text=str;
    marker_pub_.publish(text_marker);
  }

private:
  ros::NodeHandle       nh_, pnh_;
  ros::Subscriber       clicked_point_sut_;
  ros::Publisher        marker_pub_;
  tf::TransformListener tf_;
};
}//end namespace



int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_kitchen_node");
  ros::NodeHandle nh, pnh;
  spark_set_kitchen::SetKitchen set_kitchen_node(nh, pnh);
  ros::spin();
  return 0;
}//EOF

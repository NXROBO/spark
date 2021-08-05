/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, NXROBO Ltd.
 *  Litian Zhuang <litian.zhuang@nxrobo.com> and Xiankai Chen <xiankai.chen@nxrobo.com>
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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>    // odom
#include <geometry_msgs/Twist.h>  // cmd_vel
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <time.h>
#include <sensor_msgs/Imu.h>
#include "spark_base/kfilter.hpp"
#include "spark_base/mylock.hpp"
#include "spark_base/spark_base_interface.h"
#include "spark_base/SparkBaseSensor.h"
#include "spark_base/SparkBaseDock.h"

#include "spark_base/GyroMessage.h"
#include "spark_base/SparkBaseOdom.h"

using namespace std;

nxsparkbase::CMutex g_Lock;

#define NODE_VERSION 0.01
#define SPARKBASETIMEOUT (100 * 1e6)
#define COUNT_TIMES 5
#define COUNT_FREQ 50

class ComDealDataNode
{
private:
    ros::Time current_time;
    ros::Time last_time;
    std::string base_frame_id;
    std::string odom_frame_id;
    ros::Publisher odom_pub;
    ros::Publisher rb_sensor_pub;
    ros::Publisher rb_dock_pub;
    tf::TransformBroadcaster tf_broadcaster;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher gyro_pub;
    ros::Publisher fback_cmd_vel_pub;
    ros::Subscriber odom_reset_sub;
    ros::NodeHandle n;
    std::string port;
    std::string serial_port;
    ros::Timer stimer;
    ros::Subscriber dock_sub;
    ros::Subscriber search_sub;
    ros::Publisher pub_imu;
    bool vel_rt;
    double last_x, last_y, last_yaw;
    double vel_x, vel_y, vel_yaw;
    double dt;
    int idx;
    unsigned int countSerial, lastCountSerial;
    double fb_time[COUNT_TIMES], fb_dist[COUNT_TIMES], fb_dist_x[COUNT_TIMES], odom_x[COUNT_TIMES], odom_y[COUNT_TIMES],
    odom_yaw[COUNT_TIMES], vel_x_list[COUNT_TIMES], vel_y_list[COUNT_TIMES];

    double robot_yaw;

    nxsparkbase::KFilter odom_x_kfilter, odom_y_kfilter;

    ros::Publisher wheel_joint_pub;
    double left_wheel_position, right_wheel_position;

public:
    /**
   * 	the spark_base interface
   */
    nxsparkbase::OpenInterface *sparkbase;
    /**
   * 	Constructor
   */
    ComDealDataNode(ros::NodeHandle _n, const char *new_serial_port);
    /**
   * 	Destructor
   */
    ~ComDealDataNode();
    /**
   * 	To deal the buffer and public all kinds of message, such as odom, cliff msg and so on.
   */
    void dealMessageSwitch(unsigned char *recvbuf);

    void dealMessageSwitch_backup(unsigned char *recvbuf);

    /**
   * 	Init the com.
   */
    void startComParamInit();

    /**
   * 	Received the cmd_vel message
   */
    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr &cmd_vel);

    /**
   * 	reset the odom message as specific values
   */
    void resetOdomCb(const spark_base::SparkBaseOdom::ConstPtr &odom);

    /**
   * 	To check the serial port
   */
    void checkSerialGoon(const ros::TimerEvent &event);

    /**
   * 	public wheel joint states used by tf
   */
    int pubWheelJointStates(double linear_speed, double angular_speed);

    /**
   *    public joint states
   */
    int pubGyroMessage(unsigned char *buf, int len);

    /**
   *    Received the handleGoDock
   */
    void handleGoDock(const std_msgs::String::ConstPtr &msg);

    /**
   * 	Received the handleSearchDock
   */
    void handleSearchDock(const std_msgs::String::ConstPtr &msg);
};

ComDealDataNode *cddn = NULL;

std::string prefixTopic(std::string prefix, char *name)
{
    std::string topic_name = prefix;
    topic_name.append(name);

    return topic_name;
}

void ComDealDataNode::resetOdomCb(const spark_base::SparkBaseOdom::ConstPtr &odom)
{
    // ROS_INFO("cmdVelReceived ---.");
    sparkbase->setOdometry(odom->x, odom->y, odom->yaw);
    printf("setting odom!\n");
}

void ComDealDataNode::cmdVelReceived(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    // ROS_INFO("cmdVelReceived ---.");
    sparkbase->drive(cmd_vel->linear.x, cmd_vel->angular.z);
    countSerial++;
}

union Char2Float
{
    float value;
    unsigned char buffer[4];
};

int ComDealDataNode::pubGyroMessage(unsigned char *buf, int len)
{
    spark_base::GyroMessage gyro;
    float acvx,acvy,acvz,anvx,anvy,anvz,roll,pitch,yaw;
    sensor_msgs::Imu car_imu;
    tf::Quaternion q;
    Char2Float getvalue[12];
    if (len < 48)
        return -1;
    memcpy(&getvalue, buf, sizeof(float) * 12);
    gyro.acvx = getvalue[0].value;
    gyro.acvy = getvalue[1].value;
    gyro.acvz = getvalue[2].value;
    gyro.anvx = getvalue[3].value;
    gyro.anvy = getvalue[4].value;
    gyro.anvz = getvalue[5].value;
    gyro.bx = getvalue[6].value;
    gyro.by = getvalue[7].value;
    gyro.bz = getvalue[8].value;

    gyro.roll = getvalue[9].value;

    gyro.pitch = getvalue[10].value;

    gyro.yaw = getvalue[11].value;

    g_Lock.Lock();
    robot_yaw = gyro.yaw / 180 * 3.1415926535898;
    g_Lock.Unlock();

    acvx = (gyro.acvx *9.8);// m/s^2
    acvy = (gyro.acvy*9.8);
    acvz = (gyro.acvz*9.8);

    anvx = gyro.anvx;
    anvy = gyro.anvy;
    anvz = gyro.anvz;

    roll  = gyro.roll/180*M_PI;
    pitch =  gyro.pitch /180*M_PI;
    yaw   = gyro.yaw /180*M_PI;

    q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    car_imu.orientation.x = q.x();
    car_imu.orientation.y = q.y();
    car_imu.orientation.z = q.z();
    car_imu.orientation.w = q.w();
    car_imu.orientation_covariance[0] = pow(0.0017, 2);//
    car_imu.orientation_covariance[4] = pow(0.0017, 2);
    car_imu.orientation_covariance[8] = pow(0.0017, 2);


    car_imu.angular_velocity.x = anvx*M_PI/180.0;// rad/s
    car_imu.angular_velocity.y = anvy*M_PI/180.0;
    car_imu.angular_velocity.z = anvz*M_PI/180.0;
    car_imu.angular_velocity_covariance[0] = pow(0.1, 2);
    car_imu.angular_velocity_covariance[4] = pow(0.1, 2);
    car_imu.angular_velocity_covariance[8] = pow(0.1, 2);

    car_imu.linear_acceleration.x = acvx;// m/s^2
    car_imu.linear_acceleration.y = acvy;
    car_imu.linear_acceleration.z = acvz;
    car_imu.linear_acceleration_covariance[0] = pow(0.1, 2);
    car_imu.linear_acceleration_covariance[4] = pow(0.1, 2);
    car_imu.linear_acceleration_covariance[8] = pow(0.1, 2);

    car_imu.header.stamp = ros::Time::now();
    car_imu.header.frame_id = "IMU_link";
    pub_imu.publish(car_imu);


#if 0
    cout<<"acvx="<<gyro.acvx<<",acvy="<<gyro.acvy<<",acvz="<<gyro.acvz<<",anvx=" <<gyro.anvx<<",anvy=" <<gyro.anvy<<",anvz="<<gyro.anvz<<endl;
    cout<<"roll="<<gyro.roll<<",pitch="<<gyro.pitch<<",yaw="<<gyro.yaw<<endl;
    cout<<"bx="<<gyro.bx<<",by="<<gyro.by<<",bz="<<gyro.bz<<endl;
#endif

    gyro_pub.publish(gyro);
    return 0;
}

void ComDealDataNode::dealMessageSwitch(unsigned char *recvbuf)
{
    static bool last_touch = false;
    static bool last_plug = false;
    int curr_idx = (idx + COUNT_TIMES - 1) % COUNT_TIMES;
    static double last_odometry_yaw_;
    static double last_b_time;
    current_time = ros::Time::now();  // ros time

    fb_time[curr_idx] = sparkbase->current_time;  // set spark base time which is different from ros time
    double b_current_time = sparkbase->current_time * 0.0001;
    double odometry_x_ = sparkbase->odometry_x_;
    double odometry_y_ = sparkbase->odometry_y_;
    sparkbase->parseComInterfaceData(recvbuf, 0);
    if ((sizeof(float) * 12) == (recvbuf[2] - 22 - 3))
        pubGyroMessage(recvbuf + 22, sizeof(float) * 12);

    sparkbase->calculateOdometry();
    if (true)
    {  // use gyro's yaw
        g_Lock.Lock();
        sparkbase->odometry_yaw_ = robot_yaw;
        g_Lock.Unlock();
    }

    // first, we'll publish the transforms over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = sparkbase->odometry_x_;
    odom_trans.transform.translation.y = sparkbase->odometry_y_;
    //    ROS_DEBUG("x=%f,y=%f",sparkbase->odometry_x_,sparkbase->odometry_y_);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(sparkbase->odometry_yaw_);
    tf_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;

    // printf("%f,%f\n",sparkbase->odometry_x_,sparkbase->odometry_y_);
    // set the position
    odom.pose.pose.position.x = sparkbase->odometry_x_;
    odom.pose.pose.position.y = sparkbase->odometry_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(sparkbase->odometry_yaw_);
    double est_x = odom_x_kfilter.predict(sparkbase->wheel_dist);

    odom_x[curr_idx] = est_x;
    odom_yaw[curr_idx] = sparkbase->odometry_yaw_;

    dt = (fb_time[curr_idx] - fb_time[idx]) * 0.0001;
    vel_x_list[curr_idx] = (odom_x[curr_idx] - odom_x[idx]) / dt;
    vel_x = 0;
    for (int i = 0; i < COUNT_TIMES; i++)
    {
        vel_x += vel_x_list[i];
    }
    vel_x = vel_x / COUNT_TIMES;

    vel_y = 0;  //(odom_y[curr_idx] - odom_y[idx])/dt;

    double delodom = (odom_yaw[curr_idx] - odom_yaw[idx]);
    if (delodom > 3.14159265359)
    {
        delodom = delodom - 2 * 3.14159265359;
    }
    if (delodom < -3.14159265359)
    {
        delodom = delodom + 2 * 3.14159265359;
    }
    vel_yaw = delodom / dt;

    double tmp_dist = 0;
    fb_dist[curr_idx] = sparkbase->wheel_dist;
    for (int i = 0; i < COUNT_TIMES; i++)
    {
        tmp_dist += fb_dist[i];
    }

    double fb_x = tmp_dist / dt;

    idx = (idx + 1) % COUNT_TIMES;
    double diff_time = b_current_time-last_b_time;
    last_b_time = b_current_time;
    odom.child_frame_id = base_frame_id;
    if(vel_rt == true)
    {
        odom.twist.twist.linear.x = (sparkbase->d_x);
        odom.twist.twist.linear.y = (sparkbase->d_y);
        odom.twist.twist.angular.z = vel_yaw;

        last_odometry_yaw_ = sparkbase->odometry_yaw_;
    }
    else
    {
        odom.twist.twist.linear.x = vel_x;
        odom.twist.twist.linear.y = vel_y;
        odom.twist.twist.angular.z = vel_yaw;
    }
    // publish the odom's message

    // add covariance
    odom.pose.covariance[0] = pow(0.01, 2);
    odom.pose.covariance[1] = pow(0.05, 2);
    odom.pose.covariance[5] = pow(0.1, 2);
    odom_pub.publish(odom);

    // printf("odom x: %f\n",odom.pose.pose.position.x);
    // publish the feedback's twist message from the spark base
    fback_cmd_vel_pub.publish(odom.twist.twist);

    // publish wheel joint state
    pubWheelJointStates(sparkbase->d_x, vel_yaw);

    // publish irbumper
    spark_base::SparkBaseSensor sensor_msg;

    sensor_msg.ir_bumper_left = sparkbase->ir_bumper_[LEFT];
    sensor_msg.ir_bumper_front_left = sparkbase->ir_bumper_[FRONT_LEFT];
    sensor_msg.ir_bumper_front = sparkbase->ir_bumper_[FRONT];
    sensor_msg.ir_bumper_front_right = sparkbase->ir_bumper_[FRONT_RIGHT];
    sensor_msg.ir_bumper_right = sparkbase->ir_bumper_[RIGHT];
    sensor_msg.ir_bumper_back_left = sparkbase->ir_bumper_[BACK_LEFT];
    sensor_msg.ir_bumper_back_right = sparkbase->ir_bumper_[BACK_RIGHT];

    sensor_msg.cliff_left = sparkbase->cliff_[LEFT];
    sensor_msg.cliff_front_left = sparkbase->cliff_[FRONT_LEFT];
    sensor_msg.cliff_front_right = sparkbase->cliff_[FRONT_RIGHT];
    sensor_msg.cliff_right = sparkbase->cliff_[RIGHT];
    sensor_msg.cliff_back_left = sparkbase->cliff_[BACK_LEFT];
    sensor_msg.cliff_back_right = sparkbase->cliff_[BACK_RIGHT];

    sensor_msg.wheel_drop_left = sparkbase->wheel_drop_[LEFT];
    sensor_msg.wheel_drop_right = sparkbase->wheel_drop_[RIGHT];
    sensor_msg.wheel_over_current_left = sparkbase->wheel_over_current_[LEFT];
    sensor_msg.wheel_over_current_right = sparkbase->wheel_over_current_[RIGHT];

    rb_sensor_pub.publish(sensor_msg);

    spark_base::SparkBaseDock dock_msg;
    dock_msg.search_dock = sparkbase->search_dock_;
    dock_msg.touch_charge = sparkbase->touch_charge_;
    dock_msg.plug_charge = sparkbase->plug_charge_;
    if((last_touch!=sparkbase->touch_charge_)||(last_plug!=sparkbase->plug_charge_))
    {
        if(sparkbase->touch_charge_)
            n.setParam ("/battery/status", 1);   //
        else if(sparkbase->plug_charge_)
            n.setParam ("/battery/status", 2);   //
        else
            n.setParam ("/battery/status", 0);   //
        last_touch = sparkbase->touch_charge_;
        last_plug = sparkbase->plug_charge_;
    }
    dock_msg.dock_dir_left = sparkbase->dock_direction_[LEFT];
    dock_msg.dock_dir_right = sparkbase->dock_direction_[RIGHT];
    dock_msg.dock_dir_front = sparkbase->dock_direction_[FRONT];
    dock_msg.dock_dir_BACK = sparkbase->dock_direction_[BACK];

    rb_dock_pub.publish(dock_msg);
}

unsigned char checkSum(unsigned char *buf)
{
    unsigned char sum = 0;
    int i;
    for (i = 0; i < buf[2] - 3; i++)
    {
        sum += buf[i];
    }
    return sum;
}

void getSparkbaseComData(char *buf, int len)
{
    int i, j;
    static int count = 0;
    long long timediff;
    unsigned char tmpbuf[2550];
    static unsigned char recvbuf[2550];
    ros::Time currenttime;
    static ros::Time headertime;
    static int firsttime = 1;
    if (firsttime)
    {
        headertime = ros::Time::now();
        firsttime = 0;
    }
    currenttime = ros::Time::now();

    if (count == 0)
    {
        headertime = currenttime;
    }
    timediff = (currenttime - headertime).toNSec();

    if (timediff > SPARKBASETIMEOUT)
    {
        count = 0;
        ROS_ERROR("nx-base time out-%lld\n", timediff);
        headertime = currenttime;
    }
    if ((len + count) > 2048)
    {
        count = 0;
        ROS_ERROR("nx-base receive data too long!");
        return;
    }
    memcpy(recvbuf + count, buf, len);
    count += len;
BACKCHECK:
    if (count > 2)
    {
        int checkcount = count - 1;
        for (i = 0; i < checkcount; i++)
        {
            if ((recvbuf[i] == 0x53) && (recvbuf[i + 1] == 0x4b))
            {
                if (i > 0)
                {
                    count = count - i;
                    memcpy(tmpbuf, recvbuf + i, count);
                    memcpy(recvbuf, tmpbuf, count);
                }
                break;
            }
        }
#if 0
        if(i!=0)
        {
            for(j=0;j<count; j++)
                printf(L_GREEN "%02X " NONE ,(unsigned char)recvbuf[j]);    //
            printf("\n");
        }
#endif
        if (i == checkcount)
        {
            if (recvbuf[checkcount] == 0x53)
            {
                count = 1;
                recvbuf[0] = 0x53;
            }
            else
            {
                count = 0;
            }
        }
        if (count > 3)
        {
            unsigned int framelen = recvbuf[2];
            if (recvbuf[2] < 6)
            {
                count = 0;
            }
            else
            {
                if (count >= framelen)
                {
#if 0
                    for(j=0;j<framelen; j++)
                        printf("%02X ",(unsigned char)recvbuf[j]);
                    printf("\n");
#endif

                    if ((recvbuf[0] == 0x53) && (recvbuf[1] == 0x4b) && (recvbuf[framelen - 2] == 0x0d) &&
                            (recvbuf[framelen - 1] == 0x0a))  // check the header and end
                    {
                        if (checkSum(recvbuf) == recvbuf[framelen - 3])
                        {
                            if ((recvbuf[3] == 0x00) && (recvbuf[4] == 0x59))  // inquire
                            {
                                cddn->dealMessageSwitch(recvbuf);
                            }
                        }
                        else
                        {
                            //             ROS_ERROR("sparkbase-check sum error");
#if 0
                            for(j=0;j<framelen; j++)
                                printf(L_RED "%02X " NONE ,(unsigned char)recvbuf[j]);
                            printf("\n");
#endif
                        }
                    }
                    else
                    {
                        //            ROS_ERROR("sparkbase-header or ender error error");
#if 0
                        for(j=0; j<framelen; j++)
                            printf(L_RED "%02X " NONE ,(unsigned char)recvbuf[j]);
                        printf("\n");
#endif
                    }
                    if (count > framelen)
                    {
                        memcpy(tmpbuf, recvbuf + framelen, count - framelen);
                        memcpy(recvbuf, tmpbuf, count - framelen);
                        count = count - framelen;
                        headertime = currenttime;
                        goto BACKCHECK;
                    }
                    count = 0;
                }
            }
        }
    }
}

void ComDealDataNode::checkSerialGoon(const ros::TimerEvent &event)
{
    if (countSerial == lastCountSerial)
    {
        sparkbase->drive(0, 0);
    }
    else
    {
        lastCountSerial = countSerial;
    }
}

ComDealDataNode::ComDealDataNode(ros::NodeHandle _n, const char *new_serial_port)
{
    ros::NodeHandle pn("~");
    n = _n;
    g_Lock.Lock();
    robot_yaw = 0;
    g_Lock.Unlock();
    serial_port = new_serial_port;
    vel_rt = false;
    if(_n.getParam("vel_rt", vel_rt))
    {
        ROS_WARN("vel_rt is %d",vel_rt);
    }

    pn.param<std::string>("port", port, serial_port);
    pn.param<std::string>("base_frame_id", base_frame_id, "base_footprint");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    this->cmd_vel_sub = this->n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &ComDealDataNode::cmdVelReceived, this);
    this->pub_imu = this->n.advertise<sensor_msgs::Imu>("/imu_data", 1);
    this->odom_pub = this->n.advertise<nav_msgs::Odometry>("/odom", 1);
    this->fback_cmd_vel_pub =
            this->n.advertise<geometry_msgs::Twist>("/spark_base/command/velocity", 1);  // the velocity of robot's feedback
    this->odom_reset_sub =
            this->n.subscribe<spark_base::SparkBaseOdom>("/spark_base/odom/reset", 1, &ComDealDataNode::resetOdomCb, this);
    this->gyro_pub = n.advertise<spark_base::GyroMessage>("/spark_base/gyro", 1);
    this->rb_sensor_pub = this->n.advertise<spark_base::SparkBaseSensor>("/spark_base/sensor", 1);  // ir_bumper_cliff
    this->rb_dock_pub = this->n.advertise<spark_base::SparkBaseDock>("/spark_base/dock", 1);
    this->dock_sub = this->n.subscribe("/spark_base/handle_go2dock", 1, &ComDealDataNode::handleGoDock, this);
    this->search_sub = this->n.subscribe("/spark_base/handle_search_dock", 1, &ComDealDataNode::handleSearchDock, this);
    this->current_time = ros::Time::now();
    this->last_time = ros::Time::now();
    stimer = n.createTimer(ros::Duration(1), &ComDealDataNode::checkSerialGoon, this);

    idx = 0;
    for (int x = 0; x < COUNT_TIMES; x++)
    {
        fb_time[x] = 0;
        odom_x[x] = 0;
        odom_y[x] = 0;
        odom_yaw[x] = 0;
        vel_x_list[x] = 0;
    }

    // init wheel joint states
    wheel_joint_pub = n.advertise<sensor_msgs::JointState>("wheel_states", 1);
    left_wheel_position = 0;
    right_wheel_position = 0;
    // publish wheel joint state with 0,0
    pubWheelJointStates(0, 0);
}

ComDealDataNode::~ComDealDataNode()
{
    ROS_INFO("close the port.");
    sparkbase->closeSerialPort();
    if (sparkbase != NULL)
        delete sparkbase;
}
/**
 * 	Init the com
 */
void ComDealDataNode::startComParamInit()
{
    sparkbase = new nxsparkbase::OpenInterface(port.c_str());
    sparkbase->resetOdometry();
    if (sparkbase->openSerialPort(&getSparkbaseComData) == 0)
        ROS_INFO("Connected to Sparkbase.");
    else
    {
        ROS_FATAL("Could not connect to Sparkbase.");
        ROS_BREAK();
    }
}

int ComDealDataNode::pubWheelJointStates(double linear_speed, double angular_speed)
{
    int left_speed_mm_s =
            (int)((-linear_speed - SPARKBASE_AXLE_LENGTH * angular_speed / 2) * 1e3);  // Left wheel velocity in mm/s
    int right_speed_mm_s =
            (int)((-linear_speed + SPARKBASE_AXLE_LENGTH * angular_speed / 2) * 1e3);  // Right wheel velocity in mm/s
    left_wheel_position += left_speed_mm_s * 0.001 / 50 / (SPARKBASE_AXLE_LENGTH / 2);
    right_wheel_position += right_speed_mm_s * 0.001 / 50 / (SPARKBASE_AXLE_LENGTH / 2);

    while (left_wheel_position > M_PI || left_wheel_position < -M_PI || right_wheel_position > M_PI ||
           right_wheel_position <= -M_PI)
    {
        if (left_wheel_position > M_PI)
        {
            left_wheel_position -= 2 * M_PI;
        }
        else if (left_wheel_position < -M_PI)
        {
            left_wheel_position += 2 * M_PI;
        }

        if (right_wheel_position > M_PI)
        {
            right_wheel_position -= 2 * M_PI;
        }
        else if (right_wheel_position < -M_PI)
        {
            right_wheel_position += 2 * M_PI;
        }
    }

    // ROS_INFO("pleft,pright:%f,%f,%d,%d",left_wheel_position,right_wheel_position,left_speed_mm_s,right_speed_mm_s);
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] = "left_wheel_joint";
    joint_state.position[0] = left_wheel_position;
    joint_state.name[1] = "right_wheel_joint";
    joint_state.position[1] = right_wheel_position;

    wheel_joint_pub.publish(joint_state);

    return 0;
}

void ComDealDataNode::handleGoDock(const std_msgs::String::ConstPtr &msg)
{
    std::cout << "dock get string is :" << msg->data << std::endl;
    if (msg->data == "open")
        sparkbase->goDock(1);
    else
        sparkbase->goDock(0);
}

void ComDealDataNode::handleSearchDock(const std_msgs::String::ConstPtr &msg)
{
    ROS_ERROR("handleSearchDock %s", msg->data.c_str());
    if (msg->data == "open")
        sparkbase->searchDock(1);
    else
        sparkbase->searchDock(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spark_base");
    ros::NodeHandle _n;
    ROS_INFO("spark_base_node for ROS %.2f", NODE_VERSION);
    if (argc != 2)
    {
        ROS_FATAL("please input : spark_base_node /dev/sparkBase");
        ROS_BREAK();
    }
    ROS_INFO("using the %s\n", argv[1]);
    cddn = new ComDealDataNode(_n, argv[1]);
    cddn->startComParamInit();
    ros::spin();
    if (cddn != NULL)
        delete cddn;
}

// EOF

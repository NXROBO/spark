/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, NXROBO Ltd.
 *  Litian Zhuang <litian.zhuang@nxrobo.com>
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

#include "spark_base/spark_base_interface.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include <ros/ros.h>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

// *****************************************************************************
// Constructor
nxsparkbase::OpenInterface::OpenInterface(const char *new_serial_port)
{
    port_name_ = new_serial_port;

    OImode_ = OI_MODE_OFF;

    this->resetOdometry();
    this->wheel_dist = 0;

    encoder_counts_[LEFT] = 0;
    encoder_counts_[RIGHT] = 0;

    last_encoder_counts_[LEFT] = 0;
    last_encoder_counts_[RIGHT] = 0;

    is_first_time_left = true;
    is_first_time_right = true;
    serial_port_ = new cereal::CerealPort();
}

// *****************************************************************************
// Destructor
nxsparkbase::OpenInterface::~OpenInterface()
{
    // Clean up!
    delete serial_port_;
}

// *****************************************************************************
// Open the serial port
int nxsparkbase::OpenInterface::openSerialPort(boost::function<void(char *, int)> f)
{
    try
    {
        serial_port_->open(port_name_.c_str(), 115200);
    }
    catch (cereal::Exception &e)
    {
        return (-1);
    }

    this->startOI();
    if (serial_port_->startReadStream(f) != true)
    {
        closeSerialPort();
        return (-1);
    }
    return (0);
}

// *****************************************************************************
// check sum
unsigned char nxsparkbase::OpenInterface::checkSum(unsigned char *buf)
{
    unsigned char sum = 0;
    int i;
    for (i = 0; i < buf[2] - 3; i++)
    {
        sum += buf[i];
    }
    return sum;
}

// *****************************************************************************
// Set the mode
int nxsparkbase::OpenInterface::startOI(void)
{
    char buffer[8];
    buffer[0] = 0x53;                               // headcode1
    buffer[1] = 0x4b;                               // headcode2
    buffer[3] = 0x00;                               // cmd1
    buffer[4] = 0x16;                               // cmd2
    buffer[2] = 2;                                  // communicate num
    buffer[5] = checkSum((unsigned char *)buffer);  // checksum
    buffer[6] = 0x0d;                               // endcode1
    buffer[7] = 0x0a;                               // endcode2
    usleep(OI_DELAY_MODECHANGE_MS * 1e3);
    try
    {
        serial_port_->write(buffer, 8);
    }
    catch (cereal::Exception &e)
    {
        return (-1);
    }
    OImode_ = OI_MODE_FULL;
    return (0);
}

// *****************************************************************************
// Send an OP code to the spark base
int nxsparkbase::OpenInterface::sendOpcode(int code)
{
    char buffer[8];
    buffer[0] = 0x53;                               // headcode1
    buffer[1] = 0x4b;                               // headcode2
    buffer[2] = 8;                                  // communicate num
    buffer[3] = code >> 8;                          // cmd1
    buffer[4] = code & 0x00ff;                      // cmd2
    buffer[5] = checkSum((unsigned char *)buffer);  // checksum
    buffer[6] = 0x0d;                               // endcode1
    buffer[7] = 0x0a;                               // endcode2
    try
    {
        serial_port_->write(buffer, buffer[2]);
    }
    catch (cereal::Exception &e)
    {
        return (-1);
    }
    return (0);
}
// *****************************************************************************
// Send an OP code9 to the spark base
int nxsparkbase::OpenInterface::sendOpcode9(int code, unsigned char value)
{
    char buffer[9];
    buffer[0] = 0x53;                               // headcode1
    buffer[1] = 0x4b;                               // headcode2
    buffer[2] = 9;                                  // communicate num
    buffer[3] = code >> 8;                          // cmd1
    buffer[4] = code & 0x00ff;                      // cmd2
    buffer[5] = value;    			  // value
    buffer[6] = checkSum((unsigned char *)buffer);  // checksum
    buffer[7] = 0x0d;                               // endcode1
    buffer[8] = 0x0a;                               // endcode2
    try
    {
        serial_port_->write(buffer, buffer[2]);
    }
    catch (cereal::Exception &e)
    {
        return (-1);
    }
    return (0);
}
// *****************************************************************************
// Close the serial port
int nxsparkbase::OpenInterface::closeSerialPort()
{
    this->drive(0.0, 0.0);
    usleep(OI_DELAY_MODECHANGE_MS * 1e3);
    serial_port_->stopStream();
    try
    {
        serial_port_->close();
    }
    catch (cereal::Exception &e)
    {
        return (-1);
    }

    return (0);
}

// *****************************************************************************
// Set the speeds
int nxsparkbase::OpenInterface::drive(double linear_speed, double angular_speed)
{
    // int left_speed_mm_s =
    // (int)((linear_speed-SPARKBASE_AXLE_LENGTH*angular_speed/2)*1e3);		// Left
    // wheel velocity in mm/s
    // int right_speed_mm_s =
    // (int)((linear_speed+SPARKBASE_AXLE_LENGTH*angular_speed/2)*1e3);	// Right
    // wheel velocity in mm/s
    //调换了左右轮子的速度,使得半弧向后
    int left_speed_mm_s =
            (int)((linear_speed - SPARKBASE_AXLE_LENGTH * angular_speed / 2) * 1e3);  // Left wheel velocity in mm/s
    int right_speed_mm_s =
            (int)((linear_speed + SPARKBASE_AXLE_LENGTH * angular_speed / 2) * 1e3);  // Right wheel velocity in mm/s

    //    printf("%d,%d\n",left_speed_mm_s,right_speed_mm_s);
    return this->driveDirect(left_speed_mm_s, right_speed_mm_s);
}

// *****************************************************************************
// Set the motor speeds
int nxsparkbase::OpenInterface::driveDirect(int left_speed, int right_speed)
{
    // Limit velocity
    int16_t left_speed_mm_s = MAX(left_speed, -SPARKBASE_MAX_LIN_VEL_MM_S);
    left_speed_mm_s = MIN(left_speed, SPARKBASE_MAX_LIN_VEL_MM_S);
    int16_t right_speed_mm_s = MAX(right_speed, -SPARKBASE_MAX_LIN_VEL_MM_S);
    right_speed_mm_s = MIN(right_speed, SPARKBASE_MAX_LIN_VEL_MM_S);

    int16_t left_speed_pwm_s = left_speed_mm_s / SPARKBASE_PULSES_TO_MM;
    int16_t right_speed_pwm_s = right_speed_mm_s / SPARKBASE_PULSES_TO_MM;

    // printf("left_wheel_speed,right_wheel_speed,%d,%d\n",left_speed_pwm_s,right_speed_pwm_s);
    return this->drivePWM(left_speed_pwm_s, right_speed_pwm_s);
    return (0);
}

// *****************************************************************************
// Set the motor PWMs
int nxsparkbase::OpenInterface::drivePWM(int left_speed_pwm_s, int right_speed_pwm_s)
{
    // Compose comand

    char buffer[12];
    buffer[0] = 0x53;                               // headcode1
    buffer[1] = 0x4b;                               // headcode2
    buffer[2] = 0x0c;                               // 12 all communicate num
    buffer[3] = 0x00;                               // cmd1
    buffer[4] = 0x18;                               // cmd2
    buffer[5] = (char)left_speed_pwm_s;             // data1
    buffer[6] = (char)(left_speed_pwm_s >> 8);      // data2
    buffer[7] = (char)right_speed_pwm_s;            // data3
    buffer[8] = (char)(right_speed_pwm_s >> 8);     // data4
    buffer[9] = checkSum((unsigned char *)buffer);  // checksum
    buffer[10] = 0x0d;                              // endcode1
    buffer[11] = 0x0a;                              // endcode2
    /*printf("%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-\n",
         buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],
          buffer[7],buffer[8],buffer[9],buffer[10],buffer[11]);*/
    try
    {
        serial_port_->write(buffer, buffer[2]);
    }
    catch (cereal::Exception &e)
    {
        return (-1);
    }
    return (0);
}

// *****************************************************************************
// Read the sensors
int nxsparkbase::OpenInterface::getSensorPackets(int timeout)
{
    char buffer[12];
    buffer[0] = 0x53;                               // headcode1
    buffer[1] = 0x4b;                               // headcode2+6
    buffer[2] = 8;                                  // communicate num
    buffer[3] = 0x00;                               // cmd1
    buffer[4] = 0x16;                               // cmd2
    buffer[5] = checkSum((unsigned char *)buffer);  // checksum
    buffer[6] = 0x0d;                               // endcode1
    buffer[7] = 0x0a;                               // endcode2
    // Fill in the command buffer to send to the robot

    try
    {
        serial_port_->write(buffer, buffer[2]);
    }
    catch (cereal::Exception &e)
    {
        return (-1);
    }
    return (0);
}

// *****************************************************************************

int nxsparkbase::OpenInterface::parseSenseState(unsigned char *buffer, int index)
{
    // cliff, Bumps, wheeldrops
    this->cliff_[RIGHT] = (((buffer[index] >> 0) & 0x01) == 0) ? 1 : 0;
    this->cliff_[FRONT_RIGHT] = (((buffer[index] >> 1) & 0x01) == 0) ? 1 : 0;
    this->cliff_[FRONT_LEFT] = (((buffer[index] >> 2) & 0x01) == 0) ? 1 : 0;
    this->cliff_[LEFT] = (((buffer[index] >> 3) & 0x01) == 0) ? 1 : 0;
    this->cliff_[BACK_RIGHT] = (((buffer[index] >> 4) & 0x01) == 0) ? 1 : 0;
    this->cliff_[BACK_LEFT] = (((buffer[index] >> 5) & 0x01) == 0) ? 1 : 0;

    this->ir_bumper_[RIGHT] = (buffer[index + 1]) & 0x01;
    this->ir_bumper_[FRONT_RIGHT] = (buffer[index + 1] >> 1) & 0x01;
    this->ir_bumper_[FRONT] = (buffer[index + 1] >> 2) & 0x01;
    this->ir_bumper_[FRONT_LEFT] = (buffer[index + 1] >> 3) & 0x01;
    this->ir_bumper_[LEFT] = (buffer[index + 1] >> 4) & 0x01;
    this->ir_bumper_[BACK_RIGHT] = (buffer[index + 1] >> 5) & 0x01;
    this->ir_bumper_[BACK_LEFT] = (buffer[index + 1] >> 6) & 0x01;

    this->wheel_drop_[RIGHT] = (buffer[index + 2] >> 0) & 0x01;
    this->wheel_drop_[LEFT] = (buffer[index + 2] >> 1) & 0x01;
    this->wheel_over_current_[RIGHT] = (buffer[index + 2] >> 2) & 0x01;
    this->wheel_over_current_[LEFT] = (buffer[index + 2] >> 3) & 0x01;

    this->search_dock_ = (buffer[index + 2] >> 5) & 0x01;
    this->touch_charge_ = (buffer[index + 2] >> 6) & 0x01;
    this->plug_charge_ = (buffer[index + 2] >> 7) & 0x01;

    this->dock_direction_[LEFT] = (buffer[index + 3] >> 4) & 0x01;
    this->dock_direction_[FRONT] = (buffer[index + 3] >> 5) & 0x01;
    this->dock_direction_[RIGHT] = (buffer[index + 3] >> 6) & 0x01;
    this->dock_direction_[BACK] = (buffer[index + 3] >> 7) & 0x01;

    this->dock_ = ((buffer[index] >> 2) & 0x01);
    this->control_ = (buffer[index + 2] >> 4) & 0x01;
    this->error_ = (buffer[index + 1] >> 7) & 0x01;
    return (0);
}

int nxsparkbase::OpenInterface::parseRightEncoderCounts(unsigned char *buffer, int index)
{
    // Right encoder counts
    unsigned int right_encoder_counts = buffer2unsigned_int(buffer, index);
    right_encoder_counts = -right_encoder_counts;
    //    printf("Right Encoder: %d,%d,%d\n",
    //    right_encoder_counts,last_encoder_counts_[RIGHT],right_encoder_counts-last_encoder_counts_[RIGHT]);

    if (is_first_time_right ||
            right_encoder_counts == last_encoder_counts_[RIGHT])  // First time, we need 2 to make it work!
    {
        encoder_counts_[RIGHT] = 0;
        is_first_time_right = false;
    }
    else
    {
        encoder_counts_[RIGHT] = (int)(right_encoder_counts - last_encoder_counts_[RIGHT]);

        if (encoder_counts_[RIGHT] > SPARKBASE_MAX_ENCODER_COUNTS / 10)
            encoder_counts_[RIGHT] = 0; //encoder_counts_[RIGHT] - SPARKBASE_MAX_ENCODER_COUNTS;
        if (encoder_counts_[RIGHT] < -SPARKBASE_MAX_ENCODER_COUNTS / 10)
            encoder_counts_[RIGHT] = 0; //SPARKBASE_MAX_ENCODER_COUNTS + encoder_counts_[RIGHT];
    }
    /*static double rc_count = 0;
      rc_count +=   encoder_counts_[RIGHT];
    std::cout<<"sum rec:"<<rc_count<<rc_count*SPARKBASE_PULSES_TO_M<<std::endl;*/
    last_encoder_counts_[RIGHT] = right_encoder_counts;
    //    printf("Right Encoder: %d\n", encoder_counts_[RIGHT]);
    return 0;
}

int nxsparkbase::OpenInterface::parseLeftEncoderCounts(unsigned char *buffer, int index)
{
    // Left encoder counts
    unsigned int left_encoder_counts = buffer2unsigned_int(buffer, index);
    left_encoder_counts = -left_encoder_counts;
    //    printf("Left Encoder: %d,%d,%d\n", left_encoder_counts,
    //    last_encoder_counts_[LEFT],left_encoder_counts-last_encoder_counts_[LEFT]);

    if (is_first_time_left ||
            left_encoder_counts == last_encoder_counts_[LEFT])  // First time, we need 2 to make it work!
    {
        encoder_counts_[LEFT] = 0;
        is_first_time_left = false;
    }
    else
    {
        encoder_counts_[LEFT] = (int)(left_encoder_counts - last_encoder_counts_[LEFT]);

        if (encoder_counts_[LEFT] > SPARKBASE_MAX_ENCODER_COUNTS / 10)
            encoder_counts_[LEFT] = 0; //encoder_counts_[LEFT] - SPARKBASE_MAX_ENCODER_COUNTS;
        if (encoder_counts_[LEFT] < -SPARKBASE_MAX_ENCODER_COUNTS / 10)
            encoder_counts_[LEFT] = 0; //SPARKBASE_MAX_ENCODER_COUNTS + encoder_counts_[LEFT];
    }

    last_encoder_counts_[LEFT] = left_encoder_counts;
    //    printf("Left Encoder: %d\n", encoder_counts_[LEFT]);
    return 0;
}
int nxsparkbase::OpenInterface::parseWheelDiffTime(unsigned char *buffer, int index)
{
    current_time = buffer2unsigned_int(buffer, index);
    // printf("current_time:%d",current_time);
    return 0;
}

void nxsparkbase::OpenInterface::parseComInterfaceData(unsigned char *buf, int index)
{
    parseLeftEncoderCounts(buf, index + 14);
    parseRightEncoderCounts(buf, index + 10);
    parseSenseState(buf, index + 5);
    parseWheelDiffTime(buf, index + 18);
}
int nxsparkbase::OpenInterface::buffer2signed_int(unsigned char *buffer, int index)
{
    unsigned int unsigned_int;
    unsigned_int = buffer[index] | (buffer[index + 1] << 8) | (buffer[index + 2] << 16) | (buffer[index + 3] << 24);
    return (int)unsigned_int;
}

unsigned int nxsparkbase::OpenInterface::buffer2unsigned_int(unsigned char *buffer, int index)
{
    unsigned int unsigned_int;
    unsigned_int = buffer[index] | (buffer[index + 1] << 8) | (buffer[index + 2] << 16) | (buffer[index + 3] << 24);
    return unsigned_int;
}

// *****************************************************************************
// Calculate Sparkbase odometry
void nxsparkbase::OpenInterface::calculateOdometry()
{
    // double dist = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M +
    // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / 2.0;
    // double ang = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M -
    // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / -SPARKBASE_AXLE_LENGTH;
    //该版本特性：半弧为向前的方向
    // double dist = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M +
    // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / 2.0;
    // double ang = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M -
    // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / SPARKBASE_AXLE_LENGTH;
    //该版本特性：半弧为向后的方向

    double dist =
            ((double)encoder_counts_[RIGHT] * SPARKBASE_PULSES_TO_M + (double)encoder_counts_[LEFT] * SPARKBASE_PULSES_TO_M) /
            -2.0;
    double ang =
            ((double)encoder_counts_[RIGHT] * SPARKBASE_PULSES_TO_M - (double)encoder_counts_[LEFT] * SPARKBASE_PULSES_TO_M) /
            SPARKBASE_AXLE_LENGTH;
    this->d_x = dist * cos(odometry_yaw_)/((current_time-last_time)*0.0001);  // m
    this->d_y = dist * sin(odometry_yaw_)/((current_time-last_time)*0.0001);  // m
    this->d_ang = ang;
    last_time = current_time;
    // Update odometry
    this->odometry_x_ = this->odometry_x_ + dist * cos(odometry_yaw_);  // m
    this->odometry_y_ = this->odometry_y_ + dist * sin(odometry_yaw_);  // m
    this->odometry_yaw_ = NORMALIZE(this->odometry_yaw_ + ang);         // rad
    this->wheel_dist = this->wheel_dist + dist;
    // std::cout<<this->odometry_x_<<","<<this->odometry_y_<<","<<this->odometry_yaw_<<std::endl;
}

// *****************************************************************************
// Reset Sparkbase odometry
void nxsparkbase::OpenInterface::resetOdometry()
{
    this->setOdometry(0.0, 0.0, 0.0);
}

// *****************************************************************************
// Set Sparkbase odometry
void nxsparkbase::OpenInterface::setOdometry(double new_x, double new_y, double new_yaw)
{
    this->odometry_x_ = new_x;
    this->odometry_y_ = new_y;
    this->odometry_yaw_ = new_yaw;
}

// *****************************************************************************
// Go to the dock
int nxsparkbase::OpenInterface::goDock(int dock)
{
    int opcode;
    if(dock)
        opcode = 0x0010;
    else
        opcode = 0x0005;
    return sendOpcode(opcode);
}
// *****************************************************************************
// search to the dock
int nxsparkbase::OpenInterface::searchDock(int flag)
{
    int opcode = 0x0017;    //search cmd
    unsigned char value;
    if(flag)
        value = 0x00;
    else
        value = 0x01;
    return sendOpcode9(opcode, value);
}

// *****************************************************************************

// EOF

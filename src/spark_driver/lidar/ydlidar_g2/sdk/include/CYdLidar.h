/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>

using namespace ydlidar;

class YDLIDAR_API CYdLidar {
  PropertyBuilderByName(float, MaxRange,
                        private) ///< 设置和获取激光最大测距范围(m)
  PropertyBuilderByName(float, MinRange,
                        private) ///< 设置和获取激光最小测距范围(m)
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< 设置和获取激光最大角度, 最大值180度(度)
  PropertyBuilderByName(float, MinAngle,
                        private) ///< 设置和获取激光最小角度, 最小值-180度(度)
  PropertyBuilderByName(float, ScanFrequency,
                        private) ///< 设置和获取激光扫描频率(范围5HZ~12HZ)(HZ)
  PropertyBuilderByName(bool, FixedResolution,
                        private) ///< 设置和获取激光是否是固定角度分辨率
  PropertyBuilderByName(bool, Reversion,
                        private) ///< 设置和获取是否旋转激光180度
  PropertyBuilderByName(bool, AutoReconnect,
                        private) ///< 设置异常是否开启重新连接
  PropertyBuilderByName(int, SerialBaudrate,
                        private) ///< 设置和获取激光通讯波特率
  PropertyBuilderByName(int, AbnormalCheckCount,
                        private) ///< Maximum number of abnormal checks
  PropertyBuilderByName(std::string, SerialPort,
                        private) ///< 设置和获取激光端口号
  PropertyBuilderByName(std::vector<float>, IgnoreArray,
                        private) ///< 设置和获取激光剔除点
  PropertyBuilderByName(float, OffsetTime,
                        private) ///<


 public:
  CYdLidar(); //!< Constructor
  virtual ~CYdLidar();  //!< Destructor: turns the laser off.
  /**
   * @brief initialize
   * @return
   */
  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan, bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs

  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

  //get zero angle offset value
  float getAngleOffset() const;

  //Whether the zero offset angle is corrected?
  bool isAngleOffetCorrected() const;

 protected:
  /** Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /** Returns true if health status and device information has been obtained with the device. If it's not,
    * \return false on error.
    */
  bool  checkStatus();

  /** Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  /** Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth();

  /** Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo();

  /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
  bool checkScanFrequency();

  /** returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

  void checkCalibrationAngle(const std::string &serialNumber);

 private:
  bool    isScanning;
  int     node_counts ;
  float   m_AngleOffset;
  bool    m_isAngleOffsetCorrected;
  float   frequencyOffset;
  uint8_t Major;
  uint8_t Minjor;
  YDlidarDriver *lidarPtr;
  uint64_t node_duration;
  uint64_t last_node_time;
  int m_SampleRate;
  node_info *global_nodes;

};	// End of class


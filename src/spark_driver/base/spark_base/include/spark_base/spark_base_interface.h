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

#ifndef SPARK_BASE_SPARK_BASE_INTERFACE_H_
#define SPARK_BASE_SPARK_BASE_INTERFACE_H_

#include "cereal_port/CerealPort.h"

namespace nxsparkbase
{
// OI Modes
#define OI_MODE_OFF 0
#define OI_MODE_PASSIVE 1
#define OI_MODE_SAFE 2
#define OI_MODE_FULL 3

// Delay after mode change in ms
#define OI_DELAY_MODECHANGE_MS 20

// Charging states
#define OI_CHARGING_NO 0
#define OI_CHARGING_RECOVERY 1
#define OI_CHARGING_CHARGING 2
#define OI_CHARGING_TRICKLE 3
#define OI_CHARGING_WAITING 4
#define OI_CHARGING_ERROR 5

// IR Characters
#define FORCE_FIELD 161
#define GREEN_BUOY 164
#define GREEN_BUOY_FORCE_FIELD 165
#define RED_BUOY 168
#define RED_BUOY_FORCE_FIELD 169
#define RED_BUOY_GREEN_BUOY 172
#define RED_BUOY_GREEN_BUOY_FORCE_FIELD 173
#define VIRTUAL_WALL 162

// Positions
#define LEFT 0
#define RIGHT 1
#define FRONT_LEFT 2
#define FRONT_RIGHT 3
#define CENTER_LEFT 4
#define CENTER_RIGHT 5
#define FRONT 6
#define BACK 7
#define CENTER 8
#define BACK_LEFT 9
#define BACK_RIGHT 10
#define ENDPOS 11

#define OMNI 2
#define MAIN_BRUSH 2
#define SIDE_BRUSH 3

// Buttons
#define BUTTON_CLOCK 7
#define BUTTON_SCHEDULE 6
#define BUTTON_DAY 5
#define BUTTON_HOUR 4
#define BUTTON_MINUTE 3
#define BUTTON_DOCK 2
#define BUTTON_SPOT 1
#define BUTTON_CLEAN 0

#define STOP_STATE 0
#define CHARGING_STATE 1
#define CONTROL_STATE 2
#define ERROR_STATE 3

// Sparkbase Dimensions
#define SPARKBASE_BUMPER_X_OFFSET 0.050
#define SPARKBASE_DIAMETER 0.340
#define SPARKBASE_AXLE_LENGTH 0.266

#define SPARKBASE_MAX_LIN_VEL_MM_S 500
#define SPARKBASE_MAX_ANG_VEL_RAD_S 2
#define SPARKBASE_MAX_RADIUS_MM 2000

//! Sparkbase max encoder counts
#define SPARKBASE_MAX_ENCODER_COUNTS 0xFFFFFFF
//! Sparkbase encoder pulses to meter constant
#define SPARKBASE_PULSES_TO_M 0.00028885
#define SPARKBASE_PULSES_TO_MM 0.28885
#define MAX_PATH 32

#ifndef MIN
#define MIN(a, b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

//! OI op codes
/*!
 * Op codes for commands as specified by the Interface.
 */
typedef enum _OI_Opcode
{

  // Command opcodes
  OI_OPCODE_FORCE_DOCK = 0x0010,
  OI_OPCODE_STOP = 0x0005,
  OI_OPCODE_QUERY = 0x0016,
  OI_OPCODE_CONTROL = 0x0018,
  OI_OPCODE_RESULT = 0x0059,

} OI_Opcode;

/*! \class OpenInterface OpenInterface.h "inc/OpenInterface.h"
 *  \brief C++ class implementation of the iRobot OI.
 *
 * This class implements the iRobot Open Interface protocolor as described by
 *iRobot. Based on the Player Sparkbase driver writen by Brian Gerkey.
 */
class OpenInterface
{
public:
  //! Constructor
  /*!
   * By default the constructor will set the Sparkbase to read only the encoder
*counts (for odometry).
   *
   *  \param new_serial_port    Name of the serial port to open.
   *
*  \sa
   */
  OpenInterface(const char *new_serial_port);
  //! Destructor
  ~OpenInterface();

  //! Open the serial port
  /*!
   *  \param full_control    Whether to set the Sparkbase on OImode full or not.
   */
  int openSerialPort(boost::function<void(char *, int)> f);
  //! Close the serial port
  int closeSerialPort();

  //! Power down the Sparkbase.
  int powerDown();

  //! Read sensor packets
  /*!
  *  Requested the defined sensor packets from the Sparkbase. If you need odometry
  *and you requested encoder data you need to call calculateOdometry()
  *afterwords.
  *
  *  \param timeout		Timeout in milliseconds.
  *
  * \sa calculateOdometry()
  *
  *  \return 0 if ok, -1 otherwise.
  */
  int getSensorPackets(int timeout);

  //! Stream sensor packets. NOT TESTED
  int streamSensorPackets();
  //! Start stream. NOT TESTED
  int startStream();
  //! Stom stream. NOT TESTED
  int stopStream();

  //! Calculate Sparkbase odometry. Call after reading encoder pulses.
  void calculateOdometry();

  //! Drive
  /*!
  *  Send velocity commands to Sparkbase.
  *
  *  \param linear_speed  	Linear speed.
  *  \param angular_speed  	Angular speed.
  *
  *  \return 0 if ok, -1 otherwise.
  */
  int drive(double linear_speed, double angular_speed);
  //! Drive direct
  /*!
  *  Send velocity commands to Sparkbase.
  *
  *  \param left_speed  	Left wheel speed.
  *  \param right_speed  	Right wheel speed.
  *
  *  \return 0 if ok, -1 otherwise.
  */
  int driveDirect(int left_speed, int right_speed);
  //! Drive PWM
  /*!
  *  Set the motors pwms. NOT IMPLEMENTED
  *
  *  \param left_pwm  	Left wheel motor pwm.
  *  \param right_pwm  	Right wheel motor pwm.
  *
  *  \return 0 if ok, -1 otherwise.
  */
  int drivePWM(int left_pwm, int right_pwm);
  //! Set the Sparkbase in max cleaning mode. Returns the OImode to safe.
  int max();
  //! Set the Sparkbase in spot cleaning mode. Returns the OImode to safe.
  int spot();
  //! Sends the Sparkbase to the dock. Returns the OImode to safe.
  int goDock(int dock);
  int searchDock(int flag);
  //! check data sum
  /*!
  *  check data sum function.
  *
  *  \param buf  			Data to be checksum
  *
  *  \return unsigned char sm
  */
  unsigned char checkSum(unsigned char *buf);
  //! parseComInterfaceData
  /*!
  *  parse the Com Interface Data
  *
  *  \param buf  			Data to be parsed
  *  \param index  		which to start to parse
  *
  *  \return void
  */
  void parseComInterfaceData(unsigned char *buf, int index);
  //! Current operation mode, one of SPARKBASE_MODE_'s
  unsigned char OImode_;

  //! Sends the Sparkbase to the dock. Returns the OImode to safe.
  void resetOdometry();
  void setOdometry(double new_x, double new_y, double new_yaw);
  double wheel_dist;
  //! Sparkbase odometry x
  double odometry_x_;
  //! Sparkbase odometry y
  double odometry_y_;
  //! Sparkbase odometry yaw
  double odometry_yaw_;
  double d_x, d_y, d_ang;
  bool wall_;           //! Wall detected.
  bool virtual_wall_;   //! Virtual wall detected.
  bool cliff_[ENDPOS];  //! Cliff sensors. Indexes: LEFT FRONT_LEFT FRONT_RIGHT
  // RIGHT
  bool bumper_[ENDPOS];     //! Bumper sensors. Indexes: LEFT RIGHT
  bool ir_bumper_[ENDPOS];  //! IR bumper sensors. Indexes: LEFT FRONT_LEFT
  // CENTER_LEFT CENTER_RIGHT FRONT_RIGHT RIGHT
  bool wheel_over_current_[2];  //! Wheel current over: Indexes: LEFT RIGHT
  bool wheel_drop_[2];          //! Wheel drop sensors: Indexes: LEFT RIGHT
  bool omni_[ENDPOS];           //! omni wheel sensors: Indexes: FRONT BACK
  int wall_signal_;             //! Wall signal.
  int cliff_signal_[4];         //! CLiff sensors signal. Indexes: LEFT FRONT_LEFT
  // FRONT_RIGHT RIGHT
  int ir_bumper_signal_[6];  //! IR bumper sensors signal. Indexes: LEFT
  // FRONT_LEFT CENTER_LEFT CENTER_RIGHT FRONT_RIGHT
  // RIGHT
  unsigned char ir_char_[3];  //! IR characters received. Indexes: OMNI LEFT RIGHT

  bool buttons_[8];  //! Buttons. Indexes: BUTTON_CLOCK BUTTON_SCHEDULE
  // BUTTON_DAY BUTTON_HOUR BUTTON_MINUTE BUTTON_DOCK
  // BUTTON_SPOT BUTTON_CLEAN

  unsigned char dirt_detect_;  //! Dirt detected

  int motor_current_[4];  //! Motor current. Indexes: LEFT RIGHT MAIN_BRUSH
  // SIDE_BRUSH
  bool overcurrent_[4];  //! Motor overcurrent. Indexes: LEFT RIGHT MAIN_BRUSH
  // SIDE_BRUSH

  unsigned char charging_state_;  //! One of OI_CHARGING_'s
  bool power_cord_;               //! Whether the Sparkbase is connected to the power cord or not.
  bool dock_;                     //! Whether the Sparkbase is docked or not.
  bool control_;                  //! The spark base is control or not
  bool error_;                    //! The spark base is error or not

  float voltage_;     //! Battery voltage in volts.
  float current_;     //! Battery current in amps.
  char temperature_;  //! Battery temperature in C degrees.
  float charge_;      //! Battery charge in Ah.
  float capacity_;    //! Battery capacity in Ah

  bool search_dock_;             //! search dock
  bool touch_charge_;            //! touch charge
  bool plug_charge_;             //! plug charge
  bool dock_direction_[ENDPOS];  //! dock direction

  int stasis_;                //! 1 when the robot is going forward, 0 otherwise
  unsigned int current_time;  //! diff time.
  unsigned int last_time;  //! diff time.
private:
  //! Parse data
  /*!
  *  Data parsing function. Parses data comming from the Sparkbase.
  *
  *  \param buffer  			Data to be parsed.
  *  \param index  			index of the data buffer.
  *
  *  \return 0 if ok, -1 otherwise.
  */
  int parseSenseState(unsigned char *buffer, int index);
  int parseLeftEncoderCounts(unsigned char *buffer, int index);
  int parseRightEncoderCounts(unsigned char *buffer, int index);

  //! Buffer to signed int
  /*!
  *  Parsing helper function. Converts 4 bytes of data into a signed int value.
  *
  *  \param buffer  	Data buffer.
  *  \param index  	Position in the buffer where the value is.
  *
  *  \sa buffer2unsigned_int()
  *
  *  \return A signed int value.
  */
  int buffer2signed_int(unsigned char *buffer, int index);
  //! Buffer to unsigned int
  /*!
  *  Parsing helper function. Converts 4 bytes of data into an unsigned int
  *value.
  *
  *  \param buffer  	Data buffer.
  *  \param index  	Position in the buffer where the value is.
  *
  *  \sa buffer2signed_int()
  *
  *  \return An unsigned int value.
  */
  unsigned int buffer2unsigned_int(unsigned char *buffer, int index);

  //! Start OI
  /*!
  *  Start the OI, change to spark base to a OImode that allows control.
  *
  *  \param void
  *
  *  \return 0 if ok, -1 otherwise.
  */
  int startOI(void);

  //! Send OP code
  /*!
  *  Send an OP code to Sparkbase.
  *
  *  \param code  			OP code to send.
  *  \param value  			op value to send.
  *
  *  \return 0 if ok, -1 otherwise.
  */
  int sendOpcode9(int code, unsigned char value);	

  //! Send OP code
  /*!
  *  Send an OP code to Sparkbase.
  *
  *  \param code  			OP code to send.
  *
  *  \return 0 if ok, -1 otherwise.
  */
  int sendOpcode(int code);

  //! Serial port to which the robot is connected
  std::string port_name_;
  //! Cereal port object
  cereal::CerealPort *serial_port_;

  //! Stream variable. NOT TESTED
  bool stream_defined_;

  //! Number of packets
  int num_of_packets_;

  //! Total size of packets
  size_t packets_size_;

  //! Amount of distance travelled since last reading. Not being used, poor
  // resolution.
  int distance_;
  //! Amount of angle turned since last reading. Not being used, poor
  // resolution.
  int angle_;
  //! Delta encoder counts.
  int encoder_counts_[2];
  //! Last encoder counts reading. For odometry calculation.
  unsigned int last_encoder_counts_[2];

  int parseWheelDiffTime(unsigned char *buffer, int index);
  bool is_first_time_left, is_first_time_right;
};
}

#endif  // SPARK_BASE_SPARK_BASE_INTERFACE_H_

// EOF

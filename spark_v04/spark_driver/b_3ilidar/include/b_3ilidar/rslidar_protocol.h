/*
*  RSLIDAR System
*  Driver Interface
*
*  Copyright 2015 RS Team
*  All rights reserved.
*
*	Author: ruishi, Data:2015-12-25
*
*/

#ifndef RSLIDAR_PROTOCOL_H
#define RSLIDAR_PROTOCOL_H
// RP-Lidar Input Packets

#if defined(_WIN32)
#pragma pack(1)
#endif

// added by ruishi
//----------------------------------------
// Commands
//-----------------------------------------
#define COMM_HEAD_FLAGE 0xAA  // Frame header

#define COMM_FRAME_TYPE_ATTR 0x20      // attribute frame
#define COMM_FRAME_TYPE_RSP_ATTR 0x21  // attribute frame

#define COMM_FRAME_TYPE_CMD 0x40      // command farme

#define COMM_FRAME_TYPE_RSP_CMD 0x41  // command farme

#define COMM_FRAME_TYPE_MESSAGE 0x61  // message frame

// Parameter lengths
//-----------------------------------------
#define CMMD_SCAN_REQ_RSP_LENS (0U)
#define CMMD_MOTOR_CONTROL_REQ_lENS (1U)
#define CMMD_SET_MOTOR_RPM_REQ_LENS (2U)
#define CMMD_SET_MEASURE_UNIT_REQ_LENS (1U)
#define CMMD_RESET_REQ_LENS (0U)
#define CMMD_DEV_RSP_LENS (1U)

#define ATTR_DEV_INFO_PARAM_REQ_LENS (0U)
#define ATTR_DEV_INFO_PARAM_RSP_LENS (39U)
#define ATTR_DEV_INFO_HEALTH_RSP_LENS (1U)
#define ATTR_DEV_INFO_MOTOR_RSP_LENS (2U)

// Commmand response info
//-----------------------------------------
#define CMMD_FRAME_RSP_ERROR 0xC1
#define CMMD_FRAME_RSP_SUCCESS 0x00
#define CMMD_FRAME_RSP_CMD_ERROR 0x01
#define CMMD_FRAME_RSP_PARAM_LEN_ERROR 0x02
#define CMMD_FRAME_RSP_PARAM_ERROR 0x03
#define CMMD_FRAME_RSP_CRC_ERROR 0x04

#define ATTR_FRAME_RSP_ERROR 0xA1
#define ATTR_FRAME_RSP_CMD_ERROR 0x01
#define ATTR_FRAME_RSP_CRC_ERROR 0x02

// Motor start or stop control
//-----------------------------------------
#define CMMD_FRAME_PARAM_START_MOTOR 0x01
#define CMMD_FRAME_PARAM_STOP_MOTOR 0x00

// MACRO
//-----------------------------------------
#define MAX_SAMPLE_NUMBERS_PER_NOTE 128
#define NUMBER_OF_TEETH 16
#define NOTE_BUFFER_PING 0x00
#define NOTE_BUFFER_PONG 0x01

/////////////����֡�����ֶ���////////////////////////
typedef enum _cmd_code
{
  CMD_STOP = 0x01,              //ֹͣ
  CMD_START_SCAN,               //��ʼɨ��
  CMD_INIT_DLIS2K,              //��ʼ��DLIS2K
  CMD_WRITE_CAIL_DATA,          //д�궨���� //���Ǽ����״�
  CMD_WRITE_CAIL_DATA_TOF,      //д�궨���� //TOF�״�
  CMD_LASER_CTRL,               //���⿪�ؿ���
  CMD_DLIS2K_SAMPLE_DATA_8BIT,  //�������CCDԭʼ���ݵ�8bit
  CMD_DLIS2K_PIXEL_POS,         //���CCD������λ��
  CMD_LENS_FOCUS_MODE,          //���뾵ͷ����ģʽ
  CMD_PIXEL_POS_CAIL,           //��������λ�ñ궨
  CMD_MOTOR_WORK_CTRL,          //������
  CMD_MOTOR_DUTY_SET,           //������������ź�ռ�ձ�
  CMD_MOTOR_RPM_SET,            //�������ת��
  CMD_DEBUG_MESSAGE_EN,         //��ӡ������Ϣʹ��
  CMD_EARE_CAIL_DATA,           //�����궨����
  CMD_REGAIN_DEFAUT_SET,        //�ָ���������
  CMD_DEVICE_ADDR_SET,          //�趨�豸��ַ
  CMD_SAMPLE_RATE_SET,          //�趨��������
  CMD_DISTANCE_OFFSET,          //���ò�������ƫ����
  CMD_HIGH_VOLT_ADJUST,         //΢����ѹ
  CMD_MEAS_PRINTF_EN,           //���ò�����Ϣ��ӡʹ��
  CMD_HIGH_VOLT_RATE,           //�趨��ѹϵ��
  CMD_DISTANCE_RANGE,           //�趨������λ
  CMD_MEAS_UNIT,                //�趨������λ
  CMD_CAIL_MEAS,                //���ò��������궨����
  CMD_WIRELESS_POWER_CTRL,      //���߹��翪�ؿ���
  CMD_AUTO_MEAS,                //����ر��ϵ��Զ�����
  CMD_WRITE_FLASH,              //д���ݵ�FLASH
  CMD_WRITE_DEVICE_INFO,        //д��Ʒ�����Ϣ
  CMD_SYSTEM_RST,               //ϵͳ��λ
} CMD_CODE;

/////////////����֡�궨��////////////////////////
typedef enum _attr_code
{

  ATTR_READ_DEVICE_INFO = 0x53,  //��ȡ�豸��Ϣ
  ATTR_READ_DLIS2K_REG,          //��ȡDLIS2K�Ĵ���ֵ
  ATTR_READ_ONCE_MEAS,           //��ȡ���β���ֵ
  ATTR_READ_CAIL_DATA,           //��ȡ�궨����
  ATTR_READ_MOTOR_DUTY,          //��ȡ��������ź�ռ�ձ�
  ATTR_READ_MOTOR_RPM,           //��ȡ���ת���趨ֵ
  ATTR_READ_DEVICE_ADDR,         //��ȡ�豸��ַ
  ATTR_READ_SAMPLE_RATE,         //��ȡ��������
  ATTR_READ_DISTANCE_OFFSET,     //��ȡƫ����
  ATTR_READ_HIGH_VOLT,           //��ȡ��ѹֵ
  ATTR_READ_MEAS_KEYE_MESSAGE,   //��ȡ�����ؼ���Ϣ
  ATTR_READ_HIGHT_VOLT_RATE,     //��ȡ��ѹϵ��
  ATTR_READ_MEAS_RANGE,          //��ȡ��������
  ATTR_READ_MEAS_UNIT,           //��ȡ������λ
  ATTR_READ_MEAS_MODE,           //��ȡ����ģʽ
  ATTR_READ_FLASH_DATA,          //��FLASH����
  ATTR_READ_DEVICE_HEALTH,       //��ȡ�豸������Ϣ
} ATTR_CODE;

/////////////��Ϣ֡�궨��////////////////////////
typedef enum _message_code
{

  MESSAGE_DEVICE_ERROR = 0xA4,      //�����豸����
  MESSAGE_DLIS2K_SAMPLE_DATA_8BIT,  //����CCDԭʼ����
  MESSAGE_DLIS2K_PIXEL_POS,         //����CCD����λ��
  MESSAGE_DLIS2K_PIXEL_POS_DIS,     //����CCD����λ�ú;���
  MESSAGE_TOF_DISTANCE,             //����TOF���
  MESSAGE_LIDAR_DISTANCE,           //�������ǲ��
} MESSAGE_CODE;

// Commonds
//-----------------------------------------

/////////////�������궨��////////////////////////
typedef enum _comm_error_code
{
  executeSuccess = 0,
  cmdError,
  parmaLen_Error,
  parma_Error,
  crc16_Error,
  SIMPLE_CAIL_Error,
} COMM_ERROR_CODE;

/////////////ͨ��֡�ṹ////////////////////////////
typedef struct _comm_frame_t
{
  uint8_t frameStart;
  uint16_t frameLen;
  uint8_t addr;
  uint8_t frameType;
  uint8_t cmd;
  uint16_t paramLen;
  uint8_t paramBuf[0];
} __attribute__((packed)) COMM_FRAME_T;

typedef struct _comm_frame_head_t
{
  uint8_t frameStart;
  uint16_t frameLen;
  uint8_t addr;
} __attribute__((packed)) COMM_FRAME_HEAD_T;

// sdk applicantion interface--------------
//----------------------------------------
#define START_MOTOR true
#define STOP_MOTOR false
typedef struct _rslidar_response_devive_info_t
{
  _u8 productName[4];
  _u8 productDate[4];
  _u8 serialNum[8];
  _u8 softwareVersion[11];
  _u8 hardwareVersion[3];
  _u8 manufacturerInfo[3];
  _u8 gearNum[3];
  _u8 measureRange[3];
} __attribute__((packed)) RSLIDAR_RESPONSE_DEVICE_INFO_T;

typedef struct _rslidar_response_health_info_t
{
  _u8 deviceHealthInfo;
} __attribute__((packed)) RSLIDAR_RESPONSE_HEALTH_INFO_T;

typedef struct _rslidar_response_motor_info_t
{
  _u16 motorSpeed;
} __attribute__((packed)) RSLIDAR_RESPONSE_MOTOR_INFO_T;

typedef struct _rslidar_response_meature_unit_t
{
  _u16 meatureUint;
} __attribute__((packed)) RSLIDAR_RESPONSE_MEATURE_INIT_T;

typedef struct _rslidar_signal_distance_unit_t
{
  // _u8			signalValue;
  _u16 angle;
  _u16 distanceValue;
} __attribute__((packed)) RSLIDAR_SIGNAL_DISTANCE_UNIT_T;

#if defined(_WIN32)
#pragma pack()
#endif

#endif

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

#ifndef RSLIDAR_DRIVER_SERIAL_H
#define RSLIDAR_DRIVER_SERIAL_H

#define GET_SET_BUF_SIZE 128
namespace rs
{
namespace standalone
{
namespace rslidar
{
class RSlidarDriverSerialImpl : public RSlidarDriver
{
public:
  enum
  {
    MAX_SCAN_NODES = 2048,
  };

  // CRC High Byte Vaule Table
  static const uint8_t auchCRCHi[256];
  // CRC Low  Byte Vaule Table
  static const uint8_t auchCRCLo[256];

  RSlidarDriverSerialImpl();
  virtual ~RSlidarDriverSerialImpl();

public:
  virtual u_result connect(const char *port_path, _u32 baudrate, _u32 flag);
  virtual void disconnect();
  virtual bool isConnected();

  virtual u_result getErrorInfo(_u8 *eflag, _u32 timeout = DEFAULT_TIMEOUT);

  virtual u_result startScan(_u32 timeout = DEFAULT_TIMEOUT);
  virtual u_result grabScanData(RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodebuffer, size_t &count,
                                _u32 timeout = DEFAULT_TIMEOUT);
  virtual u_result ascendScanData(RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodebuffer, size_t count);

protected:
  u_result _waitNode(_u8 *node, _u32 timeout);
  u_result _waitScanData(RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodebuffer, size_t &count, _u32 timeout = DEFAULT_TIMEOUT);
  u_result _cacheScanData();
  u_result _cacheSerialData();

  u_result _sendCommand(_u8 cmd, const void *payload = NULL, size_t payloadsize = 0);
  u_result _waitResponseHeader(COMM_FRAME_T *header, size_t frameType, size_t cmdType, _u32 timeout);
  u_result checkSerialBuf();
  uint16_t _crc16(uint8_t *startByte, uint16_t numBytes);

  void _disableDataGrabbing();

  int _serial_fd;

  bool _isConnected;
  bool _isScanning;
  bool _isGetData;

  rs::hal::Locker _lock;
  rs::hal::Event _dataEvt;
  rs::hal::serial_rxtx *_rxtx;

  _u8 _flag_cached_scan_node_ping_pong;
  RSLIDAR_SIGNAL_DISTANCE_UNIT_T _cached_scan_note_bufferPing[1024];
  RSLIDAR_SIGNAL_DISTANCE_UNIT_T _cached_scan_note_bufferPong[1024];
  size_t _cached_scan_node_count;
  rs::hal::Thread _cachethread;
  rs::hal::Thread _datathread;

  _u8 _paramReceiveBuf[1024];

  pthread_mutex_t *m_lock;
  void *buffer;

  size_t error_flag;
  _u8 _error_buf[GET_SET_BUF_SIZE];
};
}
}
}

#endif

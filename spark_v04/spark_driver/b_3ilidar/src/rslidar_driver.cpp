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
#include "sdkcommon.h"
#include "abs_rxtx.h"
#include "thread.h"
#include "locker.h"
#include "event.h"
#include "rslidar_driver_serial.h"
#include "ring.h"
#include <stdio.h>

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif
#define BUF_SIZE (4 * 2 * 1024)

#define READ_BUF_SIZE (2 * 1024)
#define RING_BUF_SIZE (4 * 8 * 1024)

static volatile int buf_write_flag;
static volatile int buf_read_flag;
static volatile int buf_loop_flag;
_u8 _serialReceiveBuf[BUF_SIZE];

struct ring_buffer *ring_buf = NULL;

namespace rs
{
namespace standalone
{
namespace rslidar
{
// Factory Impl
RSlidarDriver *RSlidarDriver::CreateDriver(_u32 drivertype)
{
  switch (drivertype)
  {
    case DRIVER_TYPE_SERIALPORT:
      return new RSlidarDriverSerialImpl();
    default:
      return NULL;
  }
}
void RSlidarDriver::DisposeDriver(RSlidarDriver *drv)
{
  delete drv;
}
// CRC High Byte Vaule Table
const uint8_t RSlidarDriverSerialImpl::auchCRCHi[256] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
  0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
  0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
  0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
  0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
// CRC Low  Byte Vaule Table
const uint8_t RSlidarDriverSerialImpl::auchCRCLo[256] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D,
  0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB,
  0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2,
  0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37,
  0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8,
  0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24,
  0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63,
  0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
  0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F,
  0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
  0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C,
  0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
  0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46,
  0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
// Serial Driver Impl
RSlidarDriverSerialImpl::RSlidarDriverSerialImpl() : _isConnected(false), _isScanning(false)
{
  _rxtx = rs::hal::serial_rxtx::CreateRxTx();
  _flag_cached_scan_node_ping_pong = NOTE_BUFFER_PING;
  memset(_cached_scan_note_bufferPing, 0, sizeof(_cached_scan_note_bufferPing));
  memset(_cached_scan_note_bufferPong, 0, sizeof(_cached_scan_note_bufferPong));
  _cached_scan_node_count = 0;
  buf_write_flag = 0;
  buf_read_flag = 0;
  buf_loop_flag = 0;
  error_flag = 0;
  memset(_serialReceiveBuf, 0, sizeof(_serialReceiveBuf));

  m_lock = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
  if (pthread_mutex_init(m_lock, NULL) != 0)
  {
    fprintf(stderr, "Failed init mutex,errno:%u,reason:%s\n", errno, strerror(errno));
  }
  buffer = (void *)malloc(RING_BUF_SIZE);
  if (!buffer)
  {
    fprintf(stderr, "Failed to malloc memory.\n");
  }

  uint32_t size = RING_BUF_SIZE;
  ring_buf = ring_buffer_init(buffer, size, m_lock);
  if (!ring_buf)
  {
    fprintf(stderr, "Failed to init ring buffer.\n");
  }
}
RSlidarDriverSerialImpl::~RSlidarDriverSerialImpl()
{
  // force disconnection
  disconnect();
  ring_buffer_free(ring_buf);
  free(m_lock);
  rs::hal::serial_rxtx::ReleaseRxTx(_rxtx);
}
u_result RSlidarDriverSerialImpl::connect(const char *port_path, _u32 baudrate, _u32 flag)
{
  rs::hal::AutoLocker l(_lock);
  if (isConnected())
    return RESULT_ALREADY_DONE;
  if (!_rxtx)
    return RESULT_INSUFFICIENT_MEMORY;
  // establish the serial connection...
  if (!_rxtx->bind(port_path, baudrate) || !_rxtx->open())
  {
    printf(">>>>>>>>>>>>>>>>>>> open fail = %d \n", __LINE__);
    return RESULT_INVALID_DATA;
  }
  _rxtx->flush(0);
  _serial_fd = _rxtx->get_serialfd();
  _isConnected = true;
  _isGetData = true;
  _datathread = CLASS_THREAD(RSlidarDriverSerialImpl, _cacheSerialData);
  return RESULT_OK;
}
u_result RSlidarDriverSerialImpl::checkSerialBuf()
{
  _u16 paramLens;
  _u8 buf[GET_SET_BUF_SIZE];
  while (1)
  {
    memset(buf, 0, GET_SET_BUF_SIZE);
    if ((buf_read_flag == 0) && (buf_write_flag == BUF_SIZE))
    {
      // printf(" >>>>>>>>>>>>>>>>>>> line = %d\n",__LINE__);
      return 0;
    }
    if (buf_read_flag < buf_write_flag && (buf_read_flag + 4) < buf_write_flag)
    {
      if (_serialReceiveBuf[buf_read_flag] == COMM_HEAD_FLAGE &&
          ((_serialReceiveBuf[buf_read_flag + 4] == 0x61) || (_serialReceiveBuf[buf_read_flag + 4] == 0x41) ||
           (_serialReceiveBuf[buf_read_flag + 4] == 0xc1) || (_serialReceiveBuf[buf_read_flag + 4] == 0xa1) ||
           (_serialReceiveBuf[buf_read_flag + 4] == 0x21)))
      {
        paramLens = (_serialReceiveBuf[buf_read_flag + 1] << 8) | _serialReceiveBuf[buf_read_flag + 2];
        if (buf_read_flag + paramLens + 2 <= buf_write_flag)
        {
          memcpy(buf, &_serialReceiveBuf[buf_read_flag], paramLens + 2);
          if ((buf_read_flag + paramLens + 2) == BUF_SIZE)
          {
            buf_read_flag = 0;
          }
          else
          {
            buf_read_flag = buf_read_flag + paramLens + 2;
          }
        }
        else
        {
          break;
        }
      }
      else
      {
        //   printf(" >>>>>>>>>>>>>>>>>>> line = %d\n",__LINE__);
        buf_read_flag++;
        continue;
      }
    }
    else if (buf_read_flag > buf_write_flag && (buf_read_flag + 4 <= BUF_SIZE - 1))
    {
      if (_serialReceiveBuf[buf_read_flag] == COMM_HEAD_FLAGE &&
          ((_serialReceiveBuf[buf_read_flag + 4] == 0x61) || (_serialReceiveBuf[buf_read_flag + 4] == 0x41) ||
           (_serialReceiveBuf[buf_read_flag + 4] == 0xc1) || (_serialReceiveBuf[buf_read_flag + 4] == 0xa1) ||
           (_serialReceiveBuf[buf_read_flag + 4] == 0x21)))
      {
        paramLens = (_serialReceiveBuf[buf_read_flag + 1] << 8) | _serialReceiveBuf[buf_read_flag + 2];
        if (buf_read_flag > buf_write_flag && (buf_read_flag + paramLens + 2 < BUF_SIZE))
        {
          memcpy(buf, &_serialReceiveBuf[buf_read_flag], paramLens + 2);
          buf_read_flag = buf_read_flag + paramLens + 2;
        }
        else if ((buf_read_flag + paramLens + 2 - BUF_SIZE) <= buf_write_flag)
        {
          memcpy(buf, &_serialReceiveBuf[buf_read_flag], BUF_SIZE - buf_read_flag);
          memcpy(&buf[BUF_SIZE - buf_read_flag], _serialReceiveBuf, buf_read_flag + paramLens + 2 - BUF_SIZE);
          buf_read_flag = buf_read_flag + paramLens + 2 - BUF_SIZE;
        }
        else
        {
          break;
        }
      }
      else
      {
        buf_read_flag++;
        continue;
      }
    }
    else if (buf_read_flag > buf_write_flag && (buf_read_flag + 3 == BUF_SIZE - 1) &&
             (buf_read_flag + 4 - BUF_SIZE) < buf_write_flag)
    {
      if (_serialReceiveBuf[buf_read_flag] == COMM_HEAD_FLAGE &&
          ((_serialReceiveBuf[0] == 0x61) || (_serialReceiveBuf[0] == 0x41) || (_serialReceiveBuf[0] == 0xc1) ||
           (_serialReceiveBuf[0] == 0xa1) || (_serialReceiveBuf[0] == 0x21)))
      {
        paramLens = (_serialReceiveBuf[buf_read_flag + 1] << 8) | _serialReceiveBuf[buf_read_flag + 2];
        if (buf_read_flag + paramLens + 2 - BUF_SIZE <= buf_write_flag)
        {
          memcpy(buf, &_serialReceiveBuf[buf_read_flag], BUF_SIZE - buf_read_flag);
          memcpy(&buf[BUF_SIZE - buf_read_flag], _serialReceiveBuf, buf_read_flag + paramLens + 2 - BUF_SIZE);
          buf_read_flag = buf_read_flag + paramLens + 2 - BUF_SIZE;
        }
        else
        {
          break;
        }
      }
      else
      {
        buf_read_flag++;
        continue;
      }
    }
    else if (buf_read_flag > buf_write_flag && (buf_read_flag + 2 == BUF_SIZE - 1) &&
             (buf_read_flag + 4 - BUF_SIZE) < buf_write_flag)
    {
      if (_serialReceiveBuf[buf_read_flag] == COMM_HEAD_FLAGE &&
          ((_serialReceiveBuf[1] == 0x61) || (_serialReceiveBuf[1] == 0x41) || (_serialReceiveBuf[1] == 0xc1) ||
           (_serialReceiveBuf[1] == 0xa1) || (_serialReceiveBuf[1] == 0x21)))
      {
        paramLens = (_serialReceiveBuf[buf_read_flag + 1] << 8) | _serialReceiveBuf[buf_read_flag + 2];
        if (buf_read_flag + paramLens + 2 - BUF_SIZE <= buf_write_flag)
        {
          memcpy(buf, &_serialReceiveBuf[buf_read_flag], BUF_SIZE - buf_read_flag);
          memcpy(&buf[BUF_SIZE - buf_read_flag], _serialReceiveBuf, buf_read_flag + paramLens + 2 - BUF_SIZE);
          buf_read_flag = buf_read_flag + paramLens + 2 - BUF_SIZE;
        }
        else
        {
          break;
        }
      }
      else
      {
        buf_read_flag++;
        continue;
      }
    }
    else if (buf_read_flag > buf_write_flag && (buf_read_flag + 1 == BUF_SIZE - 1) &&
             (buf_read_flag + 4 - BUF_SIZE) < buf_write_flag)
    {
      if (_serialReceiveBuf[buf_read_flag] == COMM_HEAD_FLAGE &&
          ((_serialReceiveBuf[2] == 0x61) || (_serialReceiveBuf[2] == 0x41) || (_serialReceiveBuf[2] == 0xc1) ||
           (_serialReceiveBuf[2] == 0xa1) || (_serialReceiveBuf[2] == 0x21)))
      {
        paramLens = (_serialReceiveBuf[buf_read_flag + 1] << 8) | _serialReceiveBuf[0];
        if (buf_read_flag + paramLens + 2 - BUF_SIZE <= buf_write_flag)
        {
          memcpy(buf, &_serialReceiveBuf[buf_read_flag], BUF_SIZE - buf_read_flag);
          memcpy(&buf[BUF_SIZE - buf_read_flag], _serialReceiveBuf, buf_read_flag + paramLens + 2 - BUF_SIZE);
          buf_read_flag = buf_read_flag + paramLens + 2 - BUF_SIZE;
        }
        else
        {
          break;
        }
      }
      else
      {
        buf_read_flag++;
        continue;
      }
    }
    else if (buf_read_flag > buf_write_flag && (buf_read_flag == BUF_SIZE - 1) &&
             (buf_read_flag + 4 - BUF_SIZE) < buf_write_flag)
    {
      if (_serialReceiveBuf[buf_read_flag] == COMM_HEAD_FLAGE &&
          ((_serialReceiveBuf[3] == 0x61) || (_serialReceiveBuf[3] == 0x41) || (_serialReceiveBuf[3] == 0xc1) ||
           (_serialReceiveBuf[3] == 0xa1) || (_serialReceiveBuf[3] == 0x21)))
      {
        paramLens = (_serialReceiveBuf[0] << 8) | _serialReceiveBuf[1];
        if (buf_read_flag + paramLens + 2 - BUF_SIZE <= buf_write_flag)
        {
          memcpy(buf, &_serialReceiveBuf[buf_read_flag], BUF_SIZE - buf_read_flag);
          memcpy(&buf[BUF_SIZE - buf_read_flag], _serialReceiveBuf, buf_read_flag + paramLens + 2 - BUF_SIZE);
          buf_read_flag = buf_read_flag + paramLens + 2 - BUF_SIZE;
        }
        else
        {
          break;
        }
      }
      else
      {
        buf_read_flag = 0;
        continue;
      }
    }
    else
    {
      break;
    }

    int i;
    int ret;
    _u16 checksumCac, checksumRcv;
    checksumCac = _crc16(buf, paramLens);
    checksumRcv = *(_u16 *)(buf + paramLens);
    if (checksumCac == checksumRcv)
    {
#if 1
      if (buf[4] == 0x61 && buf[5] == 0xA9)
      {
        ring_buffer_put(ring_buf, (void *)buf, GET_SET_BUF_SIZE);
      }
      else
      {
        if (error_flag == 0)
        {
          memset(_error_buf, 0, sizeof(_error_buf));
          memcpy(_error_buf, buf, sizeof(_error_buf));
          error_flag = 1;
        }
      }
#endif
    }
    else
    {
      printf("\n");
      printf("\n");
      printf("\n");
      printf(" >>>>>>>>>>>>>>>>>>> paramLens = %d \n\n", paramLens + 2);
      printf(" >>>>>>>>>>>>>>>>>>> checksumCac  = %x   checksumRcv = %x \n", checksumCac, checksumRcv);
      for (i = 0; i < paramLens + 2; i++)
      {
        printf(" %x", buf[i]);
      }
      printf("\n");
    }
  }
}
u_result RSlidarDriverSerialImpl::_cacheSerialData()
{
  _u8 tmp[128] = { 0 };
  _u8 buf[512] = { 0 };
  // printf("   >>>>>>>>>>>>>>> %s  \n",__func__);
  static int buf_len = 0;
  _isGetData = true;
  int i;
  while (_isGetData)
  {
    usleep(5000);

    int returned_size = 0;
    int read_len;
    _u8 read_buf[READ_BUF_SIZE] = { 0 };

    if (ioctl(_serial_fd, FIONREAD, &read_len) == -1)
    {
      printf(" >>>>> get len fail !\n");
      return -1;
    }

    if (read_len > 1024)
    {
      printf("   >>>>>>>>>>>>>> line = %d  read_len = %d \n", __LINE__, read_len);
    }
    returned_size = read(_serial_fd, read_buf, read_len);
    if (buf_write_flag + returned_size >= BUF_SIZE)
    {
      memcpy(&_serialReceiveBuf[buf_write_flag], read_buf, BUF_SIZE - buf_write_flag);
      memcpy(_serialReceiveBuf, &read_buf[BUF_SIZE - buf_write_flag], buf_write_flag + returned_size - BUF_SIZE);
      buf_write_flag = buf_write_flag + returned_size - BUF_SIZE;
    }
    else if (buf_write_flag + returned_size < BUF_SIZE)
    {
      memcpy(&_serialReceiveBuf[buf_write_flag], read_buf, returned_size);
      buf_write_flag = buf_write_flag + returned_size;
    }

    checkSerialBuf();
  }
  _isGetData = false;
  return RESULT_OK;
}
void RSlidarDriverSerialImpl::disconnect()
{
  if (!_isConnected)
    return;
  _rxtx->close();
}
bool RSlidarDriverSerialImpl::isConnected()
{
  return _isConnected;
}

u_result RSlidarDriverSerialImpl::getErrorInfo(_u8 *eflag, _u32 timeout)
{
  u_result ans;
  COMM_FRAME_T *response_header = reinterpret_cast<COMM_FRAME_T *>(_paramReceiveBuf);
  if (!isConnected())
    return RESULT_OPERATION_FAIL;

  if (IS_FAIL(ans = _waitResponseHeader(response_header, 0x61, 0xAB, timeout)))
  {
    return ans;
  }

  *eflag = *(response_header->paramBuf);

  // printf(" >>>>>>>>>>>>>>>>.eflag = %x \n",*eflag);
}
u_result RSlidarDriverSerialImpl::startScan(_u32 timeout)
{
  _isScanning = true;
  _cachethread = CLASS_THREAD(RSlidarDriverSerialImpl, _cacheScanData);
  return RESULT_OK;
}
u_result RSlidarDriverSerialImpl::_cacheScanData()
{
  RSLIDAR_SIGNAL_DISTANCE_UNIT_T *local_buf = NULL;
  size_t count = NUMBER_OF_TEETH;
  size_t scan_count = 0;
  u_result ans;
  _flag_cached_scan_node_ping_pong = NOTE_BUFFER_PING;
  local_buf = _cached_scan_note_bufferPong;
  _waitScanData(local_buf, count);  // // always discard the first data since it may be incomplete
  count = NUMBER_OF_TEETH;
  while (_isScanning)
  {
    if (IS_FAIL(ans = _waitScanData(local_buf, count)))
    {
      if (!((ans == RESULT_OPERATION_TIMEOUT) || (ans == RESULT_RECEIVE_NODE_ERROR)))
      {
        _isScanning = false;
        return RESULT_OPERATION_FAIL;
      }
    }
    _lock.lock();
    _flag_cached_scan_node_ping_pong = _flag_cached_scan_node_ping_pong ^ 0x01;
    if (NOTE_BUFFER_PING == _flag_cached_scan_node_ping_pong)
    {
      local_buf = _cached_scan_note_bufferPong;
    }
    else if (NOTE_BUFFER_PONG == _flag_cached_scan_node_ping_pong)
    {
      local_buf = _cached_scan_note_bufferPing;
    }
    else
    {
    }
    _cached_scan_node_count = count;
    _dataEvt.set();
    _lock.unlock();
    count = NUMBER_OF_TEETH;
  }
  _isScanning = false;
  return RESULT_OK;
}
u_result RSlidarDriverSerialImpl::_waitScanData(RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodebuffer, size_t &count, _u32 timeout)
{
  _u8 recvBuffer[1024];
  _u16 frameLens = 0;
  _u16 paramLens = 0;
  _u16 angleRange = 0;
  _u16 angleStep = 0;
  bool flagOfNewNodes = false;
  _u16 crcCheckNum = 0;
  _u16 sampleNumber = 0;
  _u16 sampleNumberCount = 0;
  if (!_isConnected)
  {
    count = 0;
    return RESULT_OPERATION_FAIL;
  }
  size_t recvNodeCount = 0;
  _u32 startTs = getms();
  _u32 waitTime;
  u_result ans;
  while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count)
  {
    if (IS_FAIL(ans = _waitNode(recvBuffer, timeout - waitTime)))
    {
      return ans;
    }

    frameLens = (recvBuffer[1] << 8) | recvBuffer[2];
    crcCheckNum = *(_u16 *)(recvBuffer + frameLens);
    angleRange = (recvBuffer[8] << 8) | recvBuffer[9];
    if (crcCheckNum == _crc16(recvBuffer, frameLens))
    {
      paramLens = (recvBuffer[6] << 8) | recvBuffer[7];
      if (paramLens - 2 > 0)
        sampleNumber = (paramLens - 2) / 2;
      else
        sampleNumber = 21;

      if (angleRange <= 100)
      {
        flagOfNewNodes = true;
        recvNodeCount = 0;
        sampleNumberCount = 0;
        angleStep = 36000 / sampleNumber / NUMBER_OF_TEETH;
      }
      if (flagOfNewNodes)
      {
        for (int i = 0; i < sampleNumber; i++)
        {
          (nodebuffer + sampleNumberCount)->angle = angleRange + i * angleStep;
          if (paramLens - 2 > 0)
            (nodebuffer + sampleNumberCount)->distanceValue =
                (recvBuffer[10 + i * 2] << 8) | recvBuffer[10 + i * 2 + 1];
          else
            (nodebuffer + sampleNumberCount)->distanceValue = 0;
          sampleNumberCount++;
        }
        recvNodeCount++;
      }
    }
    else
    {
      printf("wait_cach:%d\n", sampleNumberCount);
      continue;
    }
    if (recvNodeCount == count)
    {
      flagOfNewNodes = false;
      count = sampleNumberCount;
      // printf(">>>>>>>>>>>>>> line = %d count =%d \n",__LINE__,count);
      return RESULT_OK;
    }
  }
  printf("wait_cach:%d\n", sampleNumberCount);
  count = sampleNumberCount;
  return RESULT_OPERATION_TIMEOUT;
}
u_result RSlidarDriverSerialImpl::_waitNode(_u8 *nodeBuffer, _u32 timeout)
{
  int recvPos = 0;
  _u32 startTs = getms();
  _u8 recvBuffer[512];
  _u32 waitTime;
  _u16 frameLens = 0;
  size_t recvSize;
  size_t remainSize;
  size_t pos;
  bool frameDataRcvFlag = false;
  while ((waitTime = getms() - startTs) <= timeout)
  {
    uint32_t len, size;
    len = ring_buffer_len(ring_buf);
    if (len > 0)
    {
      size = ring_buffer_get(ring_buf, (void *)nodeBuffer, GET_SET_BUF_SIZE);
      if (size == GET_SET_BUF_SIZE)
        return RESULT_OK;
    }
    else
    {
      usleep(5000);
    }
  }
  printf("waitnode005\n");
  return RESULT_OPERATION_TIMEOUT;
}
u_result RSlidarDriverSerialImpl::grabScanData(RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodebuffer, size_t &count, _u32 timeout)
{
  switch (_dataEvt.wait(timeout))
  {
    case rs::hal::Event::EVENT_TIMEOUT:
      count = 0;
      return RESULT_OPERATION_TIMEOUT;
      break;
    case rs::hal::Event::EVENT_OK:
    {
      rs::hal::AutoLocker l(_lock);
      size_t size_to_copy = min(count, _cached_scan_node_count);
      if (NOTE_BUFFER_PING == _flag_cached_scan_node_ping_pong)
      {
        memcpy(nodebuffer, _cached_scan_note_bufferPing, size_to_copy * sizeof(RSLIDAR_SIGNAL_DISTANCE_UNIT_T));
      }
      else if (NOTE_BUFFER_PONG == _flag_cached_scan_node_ping_pong)
      {
        memcpy(nodebuffer, _cached_scan_note_bufferPong, size_to_copy * sizeof(RSLIDAR_SIGNAL_DISTANCE_UNIT_T));
      }
      else
      {
      }
      count = size_to_copy;
      _cached_scan_node_count = 0;
    }
      return RESULT_OK;
      break;
    default:
      count = 0;
      return RESULT_OPERATION_FAIL;
      break;
  }
}
u_result RSlidarDriverSerialImpl::ascendScanData(RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodebuffer, size_t count)
{
  float inc_origin_angle = 360.0 / count;
  int i = 0;
  // Tune head
  for (i = 0; i < count; i++)
  {
    if (nodebuffer[i].distanceValue == 0)
    {
      continue;
    }
    else
    {
      while (i != 0)
      {
        i--;
        float expect_angle = (nodebuffer[i + 1].angle) / 100.0f - inc_origin_angle;
        if (expect_angle < 0.0f)
          expect_angle = 0.0f;
        nodebuffer[i].angle = (_u16)(expect_angle * 100.0f);
      }
      break;
    }
  }
  // all the data is invalid
  if (i == count)
    return RESULT_OPERATION_FAIL;
  // Tune tail
  for (i = count - 1; i >= 0; i--)
  {
    if (nodebuffer[i].distanceValue == 0)
    {
      continue;
    }
    else
    {
      while (i != (count - 1))
      {
        i++;
        float expect_angle = (nodebuffer[i - 1].angle) / 100.0f + inc_origin_angle;
        if (expect_angle > 360.0f)
          expect_angle -= 360.0f;
        nodebuffer[i].angle = (_u16)(expect_angle * 100.0f);
      }
      break;
    }
  }
  // Fill invalid angle in the scan
  float frontAngle = (nodebuffer[0].angle) / 100.0f;
  for (i = 1; i < count; i++)
  {
    if (nodebuffer[i].distanceValue == 0)
    {
      float expect_angle = frontAngle + i * inc_origin_angle;
      if (expect_angle > 360.0f)
        expect_angle -= 360.0f;
      nodebuffer[i].angle = (_u16)(expect_angle * 100.0f);
    }
  }
  // Reorder the scan according to the angle value
  for (i = 0; i < (count - 1); i++)
  {
    for (int j = (i + 1); j < count; j++)
    {
      if (nodebuffer[i].angle > nodebuffer[j].angle)
      {
        RSLIDAR_SIGNAL_DISTANCE_UNIT_T temp = nodebuffer[i];
        nodebuffer[i] = nodebuffer[j];
        nodebuffer[j] = temp;
      }
    }
  }
  return RESULT_OK;
}
u_result RSlidarDriverSerialImpl::_sendCommand(_u8 cmd, const void *payload, size_t payloadsize)
{
  _u16 checksum = 0;
  _u8 commBuf[1024] = { 0 };
  COMM_FRAME_T *commFrameTmp = reinterpret_cast<COMM_FRAME_T *>(commBuf);
  const _u8 *pPayload = reinterpret_cast<const _u8 *>(payload);
  if (!_isConnected)
    return RESULT_OPERATION_FAIL;
  commFrameTmp->frameStart = COMM_HEAD_FLAGE;
  commFrameTmp->frameLen = sizeof(COMM_FRAME_T) + payloadsize;
  commFrameTmp->frameLen = (commFrameTmp->frameLen >> 8) | (commFrameTmp->frameLen << 8);
  commFrameTmp->addr = 0;
  if (cmd >= CMD_STOP && cmd <= CMD_SYSTEM_RST)
  {
    commFrameTmp->frameType = COMM_FRAME_TYPE_CMD;
  }
  else if (cmd >= ATTR_READ_DEVICE_INFO && cmd <= ATTR_READ_DEVICE_HEALTH)
  {
    commFrameTmp->frameType = COMM_FRAME_TYPE_ATTR;
  }
  commFrameTmp->cmd = cmd;
  if (payloadsize && payload)
  {
    commFrameTmp->paramLen = payloadsize;
  }
  else
  {
    commFrameTmp->paramLen = 0;
  }
  commFrameTmp->paramLen = (commFrameTmp->paramLen >> 8) | (commFrameTmp->paramLen << 8);
  if (payloadsize > 0)
  {
    for (int i = payloadsize - 1; i >= 0; i--)
      *(commFrameTmp->paramBuf + i) = *(pPayload + i);
  }
  checksum = _crc16(commBuf, sizeof(COMM_FRAME_T) + payloadsize);
  commBuf[sizeof(COMM_FRAME_T) + payloadsize] = checksum & 0xff;
  commBuf[sizeof(COMM_FRAME_T) + payloadsize + 1] = (checksum >> 8) & 0xff;
  // send header first
  _rxtx->senddata(commBuf, sizeof(COMM_FRAME_T) + payloadsize + sizeof(_u16));
  return RESULT_OK;
}
u_result RSlidarDriverSerialImpl::_waitResponseHeader(COMM_FRAME_T *header, size_t frameType, size_t cmdType,
                                                      _u32 timeout)
{
  int recvPos = 0;
  _u32 startTs = getms();
  //_u8  recvBuffer[1024];
  _u8 *headerBuffer = reinterpret_cast<_u8 *>(header);
  // const _u8 *pParamBuff = reinterpret_cast<const _u8 *>(payload);
  _u32 waitTime;
  _u16 checksumCac, checksumRcv;
  _u8 buf[512] = { 0 };
  int i = 0;
  int _buf_write = 0;
  int _buf_read = 0;

  while ((waitTime = getms() - startTs) <= timeout)
  {
    if (1 == error_flag)
    {
      memcpy(headerBuffer, _error_buf, sizeof(_error_buf));
      error_flag = 0;
      return RESULT_OK;
    }
    else
    {
      usleep(3000);
    }
  }

  return RESULT_OPERATION_TIMEOUT;
}
//====================================================================
//    函数返回值是无符号短整型CRC值
//    待进行CRC校验计算的报文
//    待校验的报文长度
//====================================================================
uint16_t RSlidarDriverSerialImpl::_crc16(uint8_t *startByte, uint16_t numBytes)
{
  uint8_t uchCRCHi = 0xFF;  // CRC高字节的初始化
  uint8_t uchCRCLo = 0xFF;  // CRC低字节的初始化
  uint16_t uIndex;          // CRC查找表的指针
  while (numBytes--)
  {
    uIndex = uchCRCLo ^ *startByte++;  // 计算CRC
    uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
    uchCRCHi = auchCRCLo[uIndex];
  }
  return (uchCRCLo << 8 | uchCRCHi);
}

void RSlidarDriverSerialImpl::_disableDataGrabbing()
{
  _isScanning = false;
  _cachethread.join();
  _rxtx->flush(0);
}
}
}
}

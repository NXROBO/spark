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

#ifndef RSLIDAR_DRIVER_H
#define RSLIDAR_DRIVER_H

#include "rstypes.h"
#include "rslidar_protocol.h"

namespace rs
{
namespace standalone
{
namespace rslidar
{
class RSlidarDriver
{
public:
  enum
  {
    DEFAULT_TIMEOUT = 2000,  // 5000 ms
  };

  enum
  {
    DRIVER_TYPE_SERIALPORT = 0x0,
  };

public:
  /// Create an RSLIDAR Driver Instance
  /// This interface should be invoked first before any other operations
  ///
  /// \param drivertype the connection type used by the driver.
  static RSlidarDriver *CreateDriver(_u32 drivertype = DRIVER_TYPE_SERIALPORT);

  /// Dispose the RSLIDAR Driver Instance specified by the drv parameter
  /// Applications should invoke this interface when the driver instance is no longer used in order to free memory
  static void DisposeDriver(RSlidarDriver *drv);

public:
  /// Open the specified serial port and connect to a target RSLIDAR device
  ///
  /// \param port_path     the device path of the serial port
  ///        e.g. on Windows, it may be com3 or \\.\com10
  ///             on Unix-Like OS, it may be /dev/ttyS1, /dev/ttyUSB2, etc
  ///
  /// \param baudrate      the baudrate used
  ///        For most RSLIDAR models, the baudrate should be set to 115200
  ///
  /// \param flag          other flags
  ///        Reserved for future use, always set to Zero
  virtual u_result connect(const char *port_path, _u32 baudrate, _u32 flag = 0) = 0;

  /// Disconnect with the RSLIDAR and close the serial port
  virtual void disconnect() = 0;

  /// Returns TRUE when the connection has been established
  virtual bool isConnected() = 0;

  virtual u_result getErrorInfo(_u8 *eflag, _u32 timeout = DEFAULT_TIMEOUT) = 0;

  /// Start scan operation
  virtual u_result startScan(_u32 timeout = DEFAULT_TIMEOUT) = 0;

  /// Wait and grab a complete 0-360 degree scan data.
  virtual u_result grabScanData(RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodebuffer, size_t &count,
                                _u32 timeout = DEFAULT_TIMEOUT) = 0;
  virtual u_result ascendScanData(RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodebuffer, size_t count) = 0;

protected:
  RSlidarDriver()
  {
  }
  virtual ~RSlidarDriver()
  {
  }
};
}
}
}

#endif  // ARCONFIG

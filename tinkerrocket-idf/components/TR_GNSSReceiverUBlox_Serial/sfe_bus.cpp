/*
  An Arduino Library which allows you to communicate seamlessly with u-blox GNSS modules using the Configuration Interface

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/16481
  https://www.sparkfun.com/products/16344
  https://www.sparkfun.com/products/18037
  https://www.sparkfun.com/products/18719
  https://www.sparkfun.com/products/18774
  https://www.sparkfun.com/products/19663
  https://www.sparkfun.com/products/17722

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
  v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020
  v3.0 rework by Paul Clark @ SparkFun Electronics, December 8th, 2022

  https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  This library is an updated version of the popular SparkFun u-blox GNSS Arduino Library.
  v3 uses the u-blox Configuration Interface (VALSET and VALGET) to:
  detect the module (during begin); configure message intervals; configure the base location; etc..

  This version of the library will not work with older GNSS modules.
  It is specifically written for newer modules like the ZED-F9P, ZED-F9R and MAX-M10S.
  For older modules, please use v2 of the library: https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.19

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  The MIT License (MIT)
  Copyright (c) 2018 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// sfe_bus.cpp
// Modified for ESP-IDF native UART driver (no Arduino dependency)

#include "sfe_bus.h"
#include <driver/uart.h>
#include <string.h>

// Global stub Print instance so enableDebugging(Print &debugPort = Serial) links
#ifndef ARDUINO
Print Serial;
#endif

namespace SparkFun_UBLOX_GNSS
{

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Constructor
  //

  SfeSerial::SfeSerial(void) : _uartPort(UART_NUM_1), _initialized(false)
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // init() — store the UART port number.
  // The caller is responsible for having already called uart_driver_install(),
  // uart_param_config(), and uart_set_pin() before this point.

  bool SfeSerial::init(uart_port_t port)
  {
    _uartPort = port;
    _initialized = true;

    // Drain any stale bytes in the RX FIFO
    uart_flush_input(_uartPort);

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()

  uint16_t SfeSerial::available()
  {
    if (!_initialized)
      return 0;

    size_t buffered = 0;
    uart_get_buffered_data_len(_uartPort, &buffered);
    return (uint16_t)buffered;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

  uint8_t SfeSerial::writeBytes(uint8_t *dataToWrite, uint8_t length)
  {
    if (!_initialized)
      return 0;

    if (length == 0)
      return 0;

    int written = uart_write_bytes(_uartPort, dataToWrite, length);
    return (written > 0) ? (uint8_t)written : 0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

  uint8_t SfeSerial::readBytes(uint8_t *data, uint8_t length)
  {
    if (!_initialized)
      return 0;

    if (length == 0)
      return 0;

    // Non-blocking read: timeout = 0 ticks
    int read = uart_read_bytes(_uartPort, data, length, 0);
    return (read > 0) ? (uint8_t)read : 0;
  }
}

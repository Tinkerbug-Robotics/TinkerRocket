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

#include <Arduino.h>
#include "sfe_bus.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
//

namespace SparkFun_UBLOX_GNSS
{

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Constructor
  //

  SfeSerial::SfeSerial(void) : _serialPort{nullptr}
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial init()
  //
  // Methods to init/setup this device

  bool SfeSerial::init(Stream &serialPort)
  {
    // if we don't have a port already
    if (!_serialPort)
    {
      _serialPort = &serialPort;
    }

    // Get rid of any stale serial data already in the processor's RX buffer
    while (_serialPort->available())
      _serialPort->read();

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()

  uint16_t SfeSerial::available()
  {

    if (!_serialPort)
      return 0;

    return (_serialPort->available());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

  uint8_t SfeSerial::writeBytes(uint8_t *dataToWrite, uint8_t length)
  {
    if (!_serialPort)
      return 0;

    if (length == 0)
      return 0;

    return _serialPort->write(dataToWrite, length);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

  uint8_t SfeSerial::readBytes(uint8_t *data, uint8_t length)
  {
    if (!_serialPort)
      return 0;

    if (length == 0)
      return 0;

#ifdef PARTICLE
    return _serialPort->readBytes((char *)data, length);
#else
    return _serialPort->readBytes(data, length);
#endif
  }
}

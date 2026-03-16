/* Modified to remove I2C for compatability with low lever
   esp32 I2C drivers

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <Arduino.h>

#include "u-blox_GNSS.h"
#include "u-blox_external_typedefs.h"
#include "sfe_bus.h"

class SFE_UBLOX_GNSS_SERIAL : public DevUBLOXGNSS
{
public:
  SFE_UBLOX_GNSS_SERIAL() { _commType = COMM_TYPE_SERIAL; }

  ///////////////////////////////////////////////////////////////////////
  // begin()
  //
  // This method is called to initialize the SFE_UBLOX_GNSS library and connect to
  // the GNSS device. This method must be called before calling any other method
  // that interacts with the device.
  //
  // Begin will then return true if "signs of life" have been seen: reception of _any_ valid UBX packet or _any_ valid NMEA header.
  //
  // This method follows the standard startup pattern in SparkFun Arduino
  // libraries.
  //
  //  Parameter   Description
  //  ---------   ----------------------------
  //  serialPort  The Serial Stream
  //  retval      true on success, false on startup failure
  //
  bool begin(Stream &serialPort, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    // Setup Serial object and pass into the superclass
    setCommunicationBus(_serialBus);

    // Initialize the Serial bus class
    _serialBus.init(serialPort);

    // Initialize the system - return results
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }

private:
  // I2C bus class
  SparkFun_UBLOX_GNSS::SfeSerial _serialBus;
};



// #1485: the OC's side of test_imu_rate_board_parity. Compiled against
// out_computer/main/config.h for one board (TR_BOARD_* from CMake).
#include <stdint.h>
#include <SPI.h>       // host shim: SPI_MODE0, which the OC config.h names
#include "config.h"

uint16_t ocImuRateMaxHz() { return config::IMU_RATE_MAX_HZ; }
uint32_t ocI2sSampleRate() { return config::I2S_SAMPLE_RATE; }

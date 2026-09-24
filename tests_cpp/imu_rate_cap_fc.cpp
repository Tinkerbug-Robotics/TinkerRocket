// #1485: the FC's side of test_imu_rate_board_parity. Compiled against
// flight_computer/main/config.h for one board (TR_BOARD_* from CMake).
#include <stdint.h>
#include "config.h"

uint16_t fcImuRateMaxHz() { return config::IMU_RATE_MAX_HZ; }
uint32_t fcI2sSampleRate() { return config::I2S_SAMPLE_RATE; }
bool fcIsm6FifoCapture() { return config::ISM6_FIFO_CAPTURE; }

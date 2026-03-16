#ifndef CONFIG_H
#define CONFIG_H

#include <driver/i2c.h>

struct config 
{

    // ### SPI Pins ###
    static constexpr uint8_t SPI_SCK = 9;
    static constexpr uint8_t SPI_SDO = 10;
    static constexpr uint8_t SPI_SDI = 8;
    static constexpr uint32_t SPI_SPEED = 10'000'000; // 10 MHz

    // ### GNSS Serial Pins ###
    static constexpr uint8_t GNSS_RX = 3;
    static constexpr uint8_t GNSS_TX = 5;
    // Optional GNSS control pins. Set to -1 if not wired.
    static constexpr int8_t GNSS_RESET_N = -1;
    static constexpr int8_t GNSS_SAFEBOOT_N = -1;

    // ### Sensor Pins ###
    static constexpr uint8_t MMC5983MA_CS = 13;
    static constexpr uint8_t BMP585_CS = 6;
    static constexpr uint8_t ISM6HG256_CS = 7;

    // ### Sensors to use ###
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = true;
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;

    // ### Sensor Interrupt Pins ###
    static constexpr uint8_t ISM6HG256_INT = 21;
    static constexpr uint8_t BMP585_INT = 11;
    static constexpr uint8_t MMC5983MA_INT = 12;

    // ### Data Update Rates (Hz) ###
    static constexpr uint16_t FLIGHT_LOOP_UPDATE_RATE = 1200;
    static constexpr uint16_t GNSS_UPDATE_RATE = 10;
    static constexpr uint16_t BMP585_UPDATE_RATE = 500;
    static constexpr uint16_t MMC5983MA_UPDATE_RATE = 1000;
    static constexpr uint16_t ISM6HG256_UPDATE_RATE = 1200;
    static constexpr uint16_t NON_SENSOR_UPDATE_RATE = 250;
    // ISM6 full-scale configuration used by SensorConverter (and shared to OUT).
    static constexpr uint8_t ISM6_LOW_G_FS_G = 16;
    static constexpr uint16_t ISM6_HIGH_G_FS_G = 256;
    static constexpr uint16_t ISM6_GYRO_FS_DPS = 4000;

    // ### Sensor Frame Rotations ###
    // ISM6 frame -> board frame rotation (deg), CCW positive about +Z.
    // Note: board frame: +X forward, +Y left, +Z up.
    static constexpr float ISM6HG256_ROT_Z_DEG = -45.0f;
    // MMC5983MA -> board frame (deg), CCW positive about +Z.
    // Note: board frame: +X forward, +Y left, +Z up.
    static constexpr float MMC5983MA_ROT_Z_DEG = 180.0f;

    // TODO Auto dection of orientation and rotation of aribtrary board position
    // to body frame

    // ### GoPro Camera Controls (placeholder pins) ###
    // Set these to valid GPIOs for your board.
    static constexpr int8_t CAM_PWR_PIN = 30;
    static constexpr int8_t CAM_SHUTTER_PIN = 31;
    static constexpr bool USE_GOPRO = true;
    static constexpr uint16_t GOPRO_PULSE_MS = 120;

    // ### Servo Controls (placeholder pins) ###
    // Set these to valid GPIOs for your board.
    static constexpr uint8_t SERVO_PIN_1 = 43;
    static constexpr uint8_t SERVO_PIN_2 = 44;
    static constexpr uint8_t SERVO_PIN_3 = 45;
    static constexpr uint8_t SERVO_PIN_4 = 46;

    static constexpr int SERVO_BIAS_1 = 0;
    static constexpr int SERVO_BIAS_2 = 0;
    static constexpr int SERVO_BIAS_3 = 0;
    static constexpr int SERVO_BIAS_4 = 0;

    static constexpr int SERVO_HZ = 333;
    static constexpr int SERVO_MIN_US = 1000;
    static constexpr int SERVO_MAX_US = 2000;

    static constexpr float ROLL_RATE_SET_POINT = 0.0f;
    // Tuned from Rolly Poly IV flight 2026-03-08 (Kp=0.04, Ki=0.001, Kd=0.0001)
    // K_plant ≈ 345 deg/s² per deg at V≈38 m/s. PM ≈ 64° at 2.2 Hz crossover.
    static constexpr float KP = 0.04f;
    static constexpr float KI = 0.001f;
    static constexpr float KD = 0.0003f;
    static constexpr float MIN_CMD = -10.0f;
    static constexpr float MAX_CMD = 10.0f;

    static constexpr bool USE_SERVO_CONTROL = true;
    static constexpr bool SERVO_WIGGLE_ON_BOOT = true;

    // EKF pad heading: compass heading of board Z+ (deg, 0=North, 90=East)
    static constexpr float PAD_HEADING_DEG = 0.0f;

    // Cascaded angle control: outer loop P-gain (rad/s per rad)
    static constexpr float KP_ANGLE = 4.0f;
    static constexpr bool USE_ANGLE_CONTROL = false;  // default; overridden at runtime via app

    // Roll control activation delay after launch (ms).  Keeps fins neutral
    // for this many ms after launch detection before engaging any roll control.
    static constexpr uint16_t ROLL_CONTROL_DELAY_MS = 0;  // default; overridden at runtime via app

    // Gain scheduling — V_ref is the speed where base gains apply.
    // Gains scale as (V_ref/V)², capped at 3.0. At low speed, gains
    // increase to compensate for reduced aerodynamic effectiveness.
    static constexpr bool  GAIN_SCHEDULE_ENABLED = true;
    static constexpr float GAIN_SCHEDULE_V_REF = 50.0f;
    static constexpr float GAIN_SCHEDULE_V_MIN = 25.0f;

    // ### Indicators (Piezo and LED) ###
    static constexpr bool ENABLE_SOUNDS = false;
    static constexpr uint8_t PIEZO_PIN = 53;
    static constexpr uint8_t RED_LED_PIN = 2;
    static constexpr uint8_t BLUE_LED_PIN = 29;
    static constexpr uint16_t BLUE_LED_FLASH_MS = 40;
    static constexpr uint32_t HEARTBEAT_BEEP_INTERVAL_MS = 1000;
    static constexpr uint16_t HEARTBEAT_BEEP_FREQ_HZ = 2200;
    static constexpr uint16_t HEARTBEAT_BEEP_DURATION_MS = 50;
    // Set true if you also want periodic beeps during INFLIGHT.
    static constexpr bool HEARTBEAT_BEEP_IN_FLIGHT = false;

    // ### I2C Parameters ###
    static constexpr i2c_port_t ESP_I2C_PORT = I2C_NUM_0;
    static constexpr uint8_t ESP_I2C_ADR = 0x42;
    static constexpr uint8_t ESP_SDA_PIN = 41;
    static constexpr uint8_t ESP_SCL_PIN = 42;
    static constexpr uint32_t ESP_I2C_FREQ_HZ = 1'200'000; // 1.2 MHz max that is stable
    static constexpr uint16_t I2C_TX_QUEUE_LEN = 256;
 
    // ### Execution Cores ###

    // Sensor (IMU, GNSS, and power) receiver
    static constexpr uint8_t SENSOR_CORE = 0;

};

#endif // CONFIG_H

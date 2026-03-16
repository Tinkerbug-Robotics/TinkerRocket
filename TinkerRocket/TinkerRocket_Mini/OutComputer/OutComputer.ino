#include <Arduino.h>
#include <SPI.h>
#include <cstring>
#include <cmath>
#include <esp_sleep.h>
#include <Preferences.h>
#include "soc/rtc_cntl_reg.h"  // Brownout detector control
#include "freertos/queue.h"

#include "config.h"

#include <TR_I2C_Interface.h>
#include <TR_LogToFlash.h>
#include <TR_LoRa_Comms.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Coordinates.h>
#include <TR_BLE_To_APP.h>
#include <RocketComputerTypes.h>
// FlightSimulator.h removed — sim now runs on FlightComputer via TR_Sensor_Collector_Sim

static TR_I2C_Interface i2c_interface(config::I2C_PORT, config::I2C_ADDRESS);
static TR_LogToFlash logger;
static TR_BLE_To_APP ble_app("TinkerRocket");
static TR_LoRa_Comms lora_comms;
static SPIClass lora_spi(HSPI);
static SensorConverter sensor_converter;
static TR_Coordinates coord;
static volatile uint8_t pending_out_command = 0U;
static uint8_t  pending_config_data[sizeof(RollProfileData)] = {};
static volatile size_t   pending_config_data_len = 0;
static volatile uint8_t  pending_config_msg_type = 0;

// Helper: sets pending_out_command with a full memory barrier so that all
// prior writes (config data, len, msg_type) are visible to the other core
// before the command flag.  BLE callbacks run on core 0 but the main loop
// (which reads these in queueOutStatusResponse) runs on core 1.
static inline void setPendingCommand(uint8_t cmd)
{
    __sync_synchronize();  // release barrier: flush all prior writes
    pending_out_command = cmd;
}
// Command retry: the ESP32 I2C slave driver's compound-transaction bug
// can garble the response on the wire even though writeToSlave succeeds
// (it only puts bytes into the TX FIFO, not onto the bus).  Repeat each
// command for CMD_REPEAT_LIMIT polls so the FlightComputer has multiple
// chances to receive it.
static const uint8_t CMD_REPEAT_LIMIT = 5;   // ~1.25s at 250ms poll
static uint8_t cmd_delivery_count = 0;
static uint8_t cmd_delivery_id = 0;           // tracks which command the counter belongs to
static bool camera_recording_requested = false;

// Phone time sync (BLE Command 9) — used for sim file timestamps
// so each sim run gets a unique filename instead of the hardcoded
// GNSS sentinel date (2025-01-01 12:00).
static uint16_t phone_utc_year   = 0;
static uint8_t  phone_utc_month  = 0;
static uint8_t  phone_utc_day    = 0;
static uint8_t  phone_utc_hour   = 0;
static uint8_t  phone_utc_minute = 0;
static uint8_t  phone_utc_second = 0;
static uint32_t phone_sync_millis = 0;
static bool     phone_time_valid = false;

// NVS persistence for LoRa settings (config.h values are factory defaults)
static Preferences prefs;
static float   lora_freq_mhz  = config::LORA_FREQ_MHZ;
static uint8_t lora_sf         = config::LORA_SF;
static float   lora_bw_khz    = config::LORA_BW_KHZ;
static uint8_t lora_cr         = config::LORA_CR;
static int8_t  lora_tx_power   = config::LORA_TX_POWER_DBM;

static RocketState latest_rocket_state = INITIALIZATION;
static bool pwr_pin_on = false;              // Power rail state — starts OFF
static bool peripherals_initialized = false; // Deferred init for peripherals behind PWR_PIN

// Servo/PID config cache (mirrored from FlightComputer for BLE readback)
static int16_t cfg_servo_bias1 = 0;
static int16_t cfg_servo_hz    = 50;
static int16_t cfg_servo_min   = 1000;
static int16_t cfg_servo_max   = 2000;
static float   cfg_pid_kp  = 0.04f;
static float   cfg_pid_ki  = 0.001f;
static float   cfg_pid_kd  = 0.0003f;
static float   cfg_pid_min = -20.0f;
static float   cfg_pid_max = 20.0f;
static bool    cfg_servo_enabled = true;
static bool    cfg_gain_sched   = true;
static bool    cfg_use_angle_ctrl = false;
static uint16_t cfg_roll_delay_ms  = 0;

static ISM6HG256Data latest_ism6_raw = {};
static BMP585Data latest_bmp_raw = {};
static GNSSData latest_gnss_raw = {};
static POWERData latest_power_raw = {};
static NonSensorData latest_non_sensor = {};

static MMC5983MAData latest_mmc_raw = {};
static MMC5983MADataSI latest_mmc_si = {};
static bool latest_mmc_valid = false;

static bool latest_ism6_valid = false;
static bool latest_bmp_valid = false;
static GNSSDataSI latest_gnss_si = {};
static bool latest_gnss_valid = false;
static bool latest_power_valid = false;
static bool latest_non_sensor_valid = false;

static float ground_pressure_pa = 101325.0f;
static bool ground_pressure_set = false;
static float pressure_alt_m = 0.0f;
static float pressure_alt_rate_mps = 0.0f;
static float max_alt_m = 0.0f;
static float max_speed_mps = 0.0f;

static uint32_t lora_tx_ok = 0;
static uint32_t lora_tx_fail = 0;
static uint32_t last_lora_tx_ms = 0;
static bool     lora_in_rx_mode = false;
static uint32_t lora_uplink_rx_count = 0;

static OutStatusQueryData last_query_cfg = {};

static inline bool nsFlagSet(uint8_t flags, uint8_t mask)
{
    return (flags & mask) != 0U;
}

// Cached Euler angles derived from NonSensorData quaternion
static float ns_roll_deg  = 0.0f;
static float ns_pitch_deg = 0.0f;
static float ns_yaw_deg   = 0.0f;

// Convert packed quaternion (int16 * 10000) from NonSensorData to display angles.
// Quaternion convention: scalar-first [q0, q1, q2, q3], body-to-NED (FRD body).
//
// Roll uses the gimbal-lock-free body-Z azimuth method: the azimuthal angle
// of the body Z-axis projected into the NED horizontal plane.  This is the
// same formula the angle controller uses and is well-defined at all pitch
// angles, unlike the standard ZYX Euler roll which is degenerate at ±90° pitch.
// Pitch and yaw use standard Euler ZYX (both are well-behaved near vertical).
static void updateEulerFromNonSensor()
{
    if (!latest_non_sensor_valid) return;

    const float qw = (float)latest_non_sensor.q0 / 10000.0f;
    const float qx = (float)latest_non_sensor.q1 / 10000.0f;
    const float qy = (float)latest_non_sensor.q2 / 10000.0f;
    const float qz = (float)latest_non_sensor.q3 / 10000.0f;

    // Roll — gimbal-lock-free: azimuth of body Z-axis in NED horizontal plane
    float z_n = 2.0f * (qx * qz + qw * qy);
    float z_e = 2.0f * (qy * qz - qw * qx);
    ns_roll_deg = -atan2f(z_e, z_n) * (180.0f / (float)M_PI);

    // Pitch (rotation about Y) — standard Euler, well-defined at vertical
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        ns_pitch_deg = copysignf(90.0f, sinp);
    else
        ns_pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);

    // Yaw (rotation about Z) — standard Euler
    ns_yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                        1.0f - 2.0f * (qy * qy + qz * qz))
                 * (180.0f / (float)M_PI);
}

static int compareFilesDescending(const void* a, const void* b)
{
    const TR_LogFileInfo* fa = (const TR_LogFileInfo*)a;
    const TR_LogFileInfo* fb = (const TR_LogFileInfo*)b;
    return strcmp(fb->filename, fa->filename);  // Reversed for descending (most recent first)
}

static void updateDerivedAltitudeFromBMP()
{
    if (!latest_bmp_valid)
    {
        return;
    }

    BMP585DataSI bmp_si = {};
    sensor_converter.convertBMP585Data(latest_bmp_raw, bmp_si);
    const float p = bmp_si.pressure;
    if (p <= 0.0f)
    {
        return;
    }

    // Track baseline pressure while not in-flight so pressure altitude is near zero before launch.
    if (latest_rocket_state != INFLIGHT)
    {
        ground_pressure_pa = p;
        ground_pressure_set = true;
    }

    if (ground_pressure_set && ground_pressure_pa > 0.0f)
    {
        pressure_alt_m = 44330.0f * (1.0f - powf(p / ground_pressure_pa, 1.0f / 5.255f));
    }
    else
    {
        pressure_alt_m = 0.0f;
    }

    // Altitude rate now comes from FlightComputer KF via NonSensorData
    // (no local finite-difference needed).
    max_alt_m = max(max_alt_m, pressure_alt_m);
}

static void updateDerivedSpeedFromNonSensor()
{
    if (!latest_non_sensor_valid)
    {
        return;
    }

    const float e = (float)latest_non_sensor.e_vel / 100.0f;
    const float n = (float)latest_non_sensor.n_vel / 100.0f;
    const float u = (float)latest_non_sensor.u_vel / 100.0f;
    const float speed = sqrtf(e * e + n * n + u * u);

    const bool alt_apogee = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
    const bool vel_apogee = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
    if (!alt_apogee && !vel_apogee)
    {
        max_speed_mps = max(max_speed_mps, speed);
    }
}

static ISM6LowGFullScale decodeISM6LowGFS(uint8_t fs_g)
{
    switch (fs_g)
    {
        case 2:  return ISM6LowGFullScale::FS_2G;
        case 4:  return ISM6LowGFullScale::FS_4G;
        case 8:  return ISM6LowGFullScale::FS_8G;
        case 16: return ISM6LowGFullScale::FS_16G;
        default: return ISM6LowGFullScale::FS_16G;
    }
}

static ISM6HighGFullScale decodeISM6HighGFS(uint16_t fs_g)
{
    switch (fs_g)
    {
        case 32:  return ISM6HighGFullScale::FS_32G;
        case 64:  return ISM6HighGFullScale::FS_64G;
        case 128: return ISM6HighGFullScale::FS_128G;
        case 256: return ISM6HighGFullScale::FS_256G;
        default:  return ISM6HighGFullScale::FS_256G;
    }
}

static ISM6GyroFullScale decodeISM6GyroFS(uint16_t fs_dps)
{
    switch (fs_dps)
    {
        case 250:  return ISM6GyroFullScale::DPS_250;
        case 500:  return ISM6GyroFullScale::DPS_500;
        case 1000: return ISM6GyroFullScale::DPS_1000;
        case 2000: return ISM6GyroFullScale::DPS_2000;
        case 4000: return ISM6GyroFullScale::DPS_4000;
        default:   return ISM6GyroFullScale::DPS_4000;
    }
}

// I2C stream ring for framed parsing
static constexpr size_t RX_STREAM_RING = 65536;
static uint8_t rx_ring[RX_STREAM_RING];
static size_t rx_head = 0;
static size_t rx_tail = 0;
static uint32_t rx_ring_overflow_drops = 0;
static uint64_t parser_resync_drops = 0;
static uint64_t parser_len_drops = 0;

static uint32_t frames_bad_crc = 0;
static uint32_t raw_i2c_reads = 0;
static uint64_t raw_i2c_bytes = 0;
static uint32_t msg_count_query = 0;
static uint32_t msg_count_ism6 = 0;
static uint32_t msg_count_bmp = 0;
static uint32_t msg_count_mmc = 0;
static uint32_t msg_count_gnss = 0;
static uint32_t msg_count_non_sensor = 0;
static uint32_t msg_count_power = 0;
static uint32_t msg_count_start_logging = 0;
static uint32_t msg_count_end_flight = 0;
static uint32_t msg_count_unknown = 0;

static uint32_t prev_msg_count_query = 0;
static uint32_t prev_msg_count_ism6 = 0;
static uint32_t prev_msg_count_bmp = 0;
static uint32_t prev_msg_count_mmc = 0;
static uint32_t prev_msg_count_gnss = 0;
static uint32_t prev_msg_count_non_sensor = 0;
static uint32_t prev_msg_count_power = 0;
static uint32_t prev_msg_count_start_logging = 0;
static uint32_t prev_msg_count_end_flight = 0;
static uint32_t prev_msg_count_unknown = 0;
static uint32_t last_stats_ms = 0;
static uint64_t prev_bytes_rx = 0;
static uint64_t prev_bytes_nand = 0;
static uint64_t prev_raw_i2c_bytes = 0;
static uint32_t prev_ring_overruns = 0;
static uint32_t prev_ring_drop_oldest_bytes = 0;
static uint32_t interval_ring_fill_peak = 0;

static inline size_t rxLen()
{
    if (rx_head >= rx_tail) return rx_head - rx_tail;
    return RX_STREAM_RING - (rx_tail - rx_head);
}

static inline void rxPush(uint8_t b)
{
    rx_ring[rx_head] = b;
    rx_head = (rx_head + 1U) % RX_STREAM_RING;
    if (rx_head == rx_tail)
    {
        rx_ring_overflow_drops++;
        rx_tail = (rx_tail + 1U) % RX_STREAM_RING; // drop oldest on overflow
    }
}

static inline uint8_t rxPeek(size_t i)
{
    return rx_ring[(rx_tail + i) % RX_STREAM_RING];
}

static inline uint8_t rxPop()
{
    const uint8_t b = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1U) % RX_STREAM_RING;
    return b;
}

// Commands that carry a config-data payload to be read via readConfigFrame
static bool isConfigCommand(uint8_t cmd)
{
    return cmd == SIM_CONFIG_PENDING  ||
           cmd == SERVO_CONFIG_PENDING ||
           cmd == PID_CONFIG_PENDING  ||
           cmd == SERVO_TEST_PENDING ||
           cmd == ROLL_PROFILE_PENDING ||
           cmd == SERVO_REPLAY_PENDING ||
           cmd == ROLL_CTRL_CONFIG_PENDING;
}

static void queueOutStatusResponse(bool ready)
{
    const uint8_t cmd = pending_out_command;  // snapshot (volatile)
    __sync_synchronize();  // acquire: see config data written before the command

    if (cmd != 0U)
    {
        // Detect new command (different from what we were repeating)
        if (cmd != cmd_delivery_id)
        {
            cmd_delivery_count = 0;
            cmd_delivery_id = cmd;
        }
        Serial.printf("[I2C TX] StatusResponse: ready=%d cmd=0x%02X attempt=%u/%u\n",
                      ready ? 1 : 0, (unsigned)cmd,
                      (unsigned)(cmd_delivery_count + 1), (unsigned)CMD_REPEAT_LIMIT);
    }

    uint8_t payload[2] = { ready ? 1U : 0U, cmd };
    uint8_t frame[MAX_FRAME];
    size_t frame_len = 0;
    if (!TR_I2C_Interface::packMessage(OUT_STATUS_RESPONSE,
                                       payload,
                                       sizeof(payload),
                                       frame,
                                       sizeof(frame),
                                       frame_len))
    {
        return;
    }
    // Non-blocking write (timeout=0): if the TX buffer is full from stale
    // un-consumed responses, just drop this one — the next query cycle will
    // generate a fresh response.  Blocking here would stall the main loop
    // and starve serviceI2CIngress(), causing data gaps.
    i2c_interface.writeToSlave(frame, frame_len, 0);

    // Queue config data ONLY for actual config commands, and only on the
    // first delivery attempt.  Writing config data alongside non-config
    // commands (e.g. SIM_START_CMD after SIM_CONFIG_PENDING) pollutes the
    // slave TX FIFO.  The FlightComputer now reads 96 bytes per poll
    // (combined read), so accumulation is no longer an issue.
    // Write config on EVERY delivery so the FC always has a chance to
    // capture it in the same I2C transaction as the response.
    if (cmd != 0U && pending_config_data_len > 0
        && isConfigCommand(cmd))
    {
        uint8_t cfg_frame[MAX_FRAME];
        size_t  cfg_frame_len = 0;
        if (TR_I2C_Interface::packMessage(pending_config_msg_type,
                                           pending_config_data,
                                           pending_config_data_len,
                                           cfg_frame,
                                           sizeof(cfg_frame),
                                           cfg_frame_len))
        {
            int written = i2c_interface.writeToSlave(cfg_frame, cfg_frame_len, 0);
            Serial.printf("[I2C TX] Config frame type=0x%02X len=%u written=%d\n",
                          (unsigned)pending_config_msg_type,
                          (unsigned)cfg_frame_len, written);
        }
        else
        {
            Serial.printf("[I2C TX] Config pack FAILED type=0x%02X data_len=%u\n",
                          (unsigned)pending_config_msg_type,
                          (unsigned)pending_config_data_len);
        }
    }
    else if (cmd != 0U && isConfigCommand(cmd) && pending_config_data_len == 0)
    {
        // Config command but no data — should not happen
        Serial.printf("[I2C TX] WARNING: config cmd=0x%02X but data_len=0\n",
                      (unsigned)cmd);
    }

    // Repeat each command for CMD_REPEAT_LIMIT polls so the FlightComputer
    // has multiple chances to receive it.
    if (cmd != 0U)
    {
        cmd_delivery_count++;
        if (cmd_delivery_count >= CMD_REPEAT_LIMIT)
        {
            Serial.printf("[I2C TX] Cmd 0x%02X cleared after %u deliveries\n",
                          (unsigned)cmd, (unsigned)cmd_delivery_count);
            setPendingCommand(0U);
            pending_config_data_len = 0;
            pending_config_msg_type = 0;
            cmd_delivery_count = 0;
            cmd_delivery_id = 0;
        }
    }
}

static const char* rocketStateToString(RocketState s)
{
    switch (s)
    {
        case INITIALIZATION: return "INITIALIZATION";
        case READY:          return "READY";
        case PRELAUNCH:      return "PRELAUNCH";
        case INFLIGHT:       return "INFLIGHT";
        case LANDED:         return "LANDED";
        default:             return "UNKNOWN";
    }
}

// ---- Shared frame processor: used by both real I2C and sim paths ----
// Logs the frame, then dispatches by type to update latest_*_raw and derived fields.
static void processFrame(const uint8_t* frame, size_t frame_len,
                         uint8_t type, const uint8_t* payload, size_t payload_len)
{
    (void)logger.enqueueFrame(frame, frame_len); // exact bytes as received

    if (type == OUT_STATUS_QUERY)
    {
        msg_count_query++;
        if (payload_len >= sizeof(OutStatusQueryData))
        {
            memcpy(&last_query_cfg, payload, sizeof(OutStatusQueryData));
            sensor_converter.configureISM6HG256FullScale(
                decodeISM6LowGFS(last_query_cfg.ism6_low_g_fs_g),
                decodeISM6HighGFS(last_query_cfg.ism6_high_g_fs_g),
                decodeISM6GyroFS(last_query_cfg.ism6_gyro_fs_dps));
            sensor_converter.configureISM6HG256RotationZ(
                (float)last_query_cfg.ism6_rot_z_cdeg / 100.0f);
            sensor_converter.configureMMC5983MARotationZ(
                (float)last_query_cfg.mmc_rot_z_cdeg / 100.0f);
            // Apply high-g bias from FlightComputer calibration (format v2+)
            if (last_query_cfg.format_version >= 2)
            {
                sensor_converter.setHighGBias(
                    (float)last_query_cfg.hg_bias_x_cmss / 100.0f,
                    (float)last_query_cfg.hg_bias_y_cmss / 100.0f,
                    (float)last_query_cfg.hg_bias_z_cmss / 100.0f);
            }
        }
        queueOutStatusResponse(true);
    }
    else if (type == ISM6HG256_MSG)
    {
        msg_count_ism6++;
        if (payload_len >= sizeof(ISM6HG256Data))
        {
            memcpy(&latest_ism6_raw, payload, sizeof(ISM6HG256Data));
            latest_ism6_valid = true;
        }
    }
    else if (type == BMP585_MSG)
    {
        msg_count_bmp++;
        if (payload_len >= sizeof(BMP585Data))
        {
            memcpy(&latest_bmp_raw, payload, sizeof(BMP585Data));
            latest_bmp_valid = true;
            updateDerivedAltitudeFromBMP();
        }
    }
    else if (type == MMC5983MA_MSG)
    {
        msg_count_mmc++;
        if (payload_len >= sizeof(MMC5983MAData))
        {
            memcpy(&latest_mmc_raw, payload, sizeof(MMC5983MAData));
            sensor_converter.convertMMC5983MAData(latest_mmc_raw, latest_mmc_si);
            latest_mmc_valid = true;
        }
    }
    else if (type == GNSS_MSG)
    {
        msg_count_gnss++;
        if (payload_len >= sizeof(GNSSData))
        {
            memcpy(&latest_gnss_raw, payload, sizeof(GNSSData));
            sensor_converter.convertGNSSData(latest_gnss_raw, latest_gnss_si);
            latest_gnss_valid = true;
        }
    }
    else if (type == NON_SENSOR_MSG)
    {
        msg_count_non_sensor++;
        if (payload_len >= sizeof(NonSensorData))
        {
            const RocketState prev_state = latest_rocket_state;
            memcpy(&latest_non_sensor, payload, sizeof(NonSensorData));
            latest_non_sensor_valid = true;
            latest_rocket_state = (RocketState)latest_non_sensor.rocket_state;
            if (latest_rocket_state != prev_state && latest_rocket_state == PRELAUNCH)
            {
                max_alt_m = 0.0f;
                max_speed_mps = 0.0f;
                pressure_alt_rate_mps = 0.0f;
                ground_pressure_set = false;  // Re-acquire ground pressure for new flight

                // Pre-create log file now so there's no NAND stall at launch
                logger.prepareLogFile();
                Serial.println("[OUT] PRELAUNCH - pre-creating log file");
            }
            // KF-filtered altitude rate from FlightComputer (or sim equivalent)
            pressure_alt_rate_mps = (float)latest_non_sensor.baro_alt_rate_dmps * 0.1f;
            updateEulerFromNonSensor();
            updateDerivedSpeedFromNonSensor();
        }
    }
    else if (type == POWER_MSG)
    {
        msg_count_power++;
        if (payload_len >= sizeof(POWERData))
        {
            memcpy(&latest_power_raw, payload, sizeof(POWERData));
            latest_power_valid = true;
        }
    }
    else if (type == START_LOGGING)
    {
        msg_count_start_logging++;
        logger.startLogging();
    }
    else if (type == END_FLIGHT)
    {
        msg_count_end_flight++;
        logger.endLogging();
    }
    else
    {
        msg_count_unknown++;
    }
}

// ---- I2C entry point ----
static void handleReceivedFrame(const uint8_t* frame, size_t frame_len,
                                uint8_t type, const uint8_t* payload, size_t payload_len)
{
    // Sim now runs on FlightComputer — all frames (real or simulated) flow
    // through the same pipeline.  No filtering needed.
    processFrame(frame, frame_len, type, payload, payload_len);
}

static void parseRxStream()
{
    uint8_t payload[MAX_PAYLOAD];
    while (rxLen() >= (4 + 1 + 1 + 2))
    {
        if (!(rxPeek(0) == 0xAA &&
              rxPeek(1) == 0x55 &&
              rxPeek(2) == 0xAA &&
              rxPeek(3) == 0x55))
        {
            parser_resync_drops++;
            (void)rxPop();
            continue;
        }

        const size_t payload_len = rxPeek(5);
        const size_t frame_len = 4 + 1 + 1 + payload_len + 2;
        if (payload_len > MAX_PAYLOAD)
        {
            parser_len_drops++;
            (void)rxPop();
            continue;
        }
        if (rxLen() < frame_len)
        {
            return;
        }

        uint8_t frame[MAX_FRAME];
        for (size_t i = 0; i < frame_len; ++i)
        {
            frame[i] = rxPeek(i);
        }

        uint8_t type = 0;
        size_t out_payload_len = 0;
        const bool ok = TR_I2C_Interface::unpackMessage(frame,
                                                        frame_len,
                                                        type,
                                                        payload,
                                                        sizeof(payload),
                                                        out_payload_len,
                                                        true);
        if (!ok)
        {
            frames_bad_crc++;
            (void)rxPop();
            continue;
        }

        for (size_t i = 0; i < frame_len; ++i)
        {
            (void)rxPop();
        }
        handleReceivedFrame(frame, frame_len, type, payload, out_payload_len);
    }
}

static void serviceI2CIngress()
{
    uint8_t inbuf[512];
    const uint32_t start_us = micros();
    size_t bytes_processed = 0;
    while (true)
    {
        if ((micros() - start_us) >= config::I2C_INGRESS_BUDGET_US)
        {
            break;
        }
        if (bytes_processed >= config::I2C_INGRESS_BUDGET_BYTES)
        {
            break;
        }
        const int n = i2c_interface.readFromSlave(inbuf, sizeof(inbuf), 0);
        if (n <= 0)
        {
            break;
        }
        raw_i2c_reads++;
        raw_i2c_bytes += (uint64_t)n;
        bytes_processed += (size_t)n;
        for (int i = 0; i < n; ++i)
        {
            rxPush(inbuf[i]);
        }
        // Drain parser continuously so the ring does not overrun under high ingress.
        parseRxStream();
    }
    parseRxStream();
}

// ============================================================================
static bool buildLoRaPayload(uint8_t out_payload[SIZE_OF_LORA_DATA])
{
    if (out_payload == nullptr)
    {
        return false;
    }

    LoRaDataSI lora = {};
    if (latest_gnss_valid)
    {
        lora.num_sats = latest_gnss_si.num_sats;
        lora.pdop = latest_gnss_si.pdop;
        lora.horizontal_accuracy = latest_gnss_si.horizontal_accuracy;

        coord.geodeticToECEF(latest_gnss_si.lat * TR_Coordinates::DEG2RAD,
                             latest_gnss_si.lon * TR_Coordinates::DEG2RAD,
                             latest_gnss_si.alt,
                             lora.ecef_x,
                             lora.ecef_y,
                             lora.ecef_z);
    }

    lora.rocket_state = (uint8_t)latest_rocket_state;
    lora.camera_recording = camera_recording_requested;
    lora.logging_active = logger.isLoggingActive();

    if (latest_non_sensor_valid)
    {
        lora.roll = ns_roll_deg;
        lora.pitch = ns_pitch_deg;
        lora.yaw = ns_yaw_deg;
        lora.q0 = (float)latest_non_sensor.q0 / 10000.0f;
        lora.q1 = (float)latest_non_sensor.q1 / 10000.0f;
        lora.q2 = (float)latest_non_sensor.q2 / 10000.0f;
        lora.q3 = (float)latest_non_sensor.q3 / 10000.0f;
        lora.launch_flag = nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
        lora.vel_u_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
        lora.alt_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
        lora.alt_landed_flag = nsFlagSet(latest_non_sensor.flags, NSF_ALT_LANDED);

        const float e = (float)latest_non_sensor.e_vel / 100.0f;
        const float n = (float)latest_non_sensor.n_vel / 100.0f;
        const float u = (float)latest_non_sensor.u_vel / 100.0f;
        lora.speed = sqrtf(e * e + n * n + u * u);
    }

    if (latest_ism6_valid)
    {
        ISM6HG256DataSI ism_si = {};
        sensor_converter.convertISM6HG256Data(latest_ism6_raw, ism_si);
        lora.acc_x = (float)ism_si.low_g_acc_x;
        lora.acc_y = (float)ism_si.low_g_acc_y;
        lora.acc_z = (float)ism_si.low_g_acc_z;
        lora.gyro_x = (float)ism_si.gyro_x;
        lora.gyro_y = (float)ism_si.gyro_y;
        lora.gyro_z = (float)ism_si.gyro_z;
        lora.temp = 0.0f;
    }

    if (latest_power_valid)
    {
        POWERDataSI power_si = {};
        sensor_converter.convertPowerData(latest_power_raw, power_si);
        lora.voltage = power_si.voltage;
        lora.current = power_si.current;
        lora.soc = power_si.soc;
    }

    lora.pressure_alt = pressure_alt_m;
    lora.altitude_rate = pressure_alt_rate_mps;
    lora.max_alt = max_alt_m;
    lora.max_speed = max_speed_mps;

    lora.base_station_voltage = 0.0f;
    lora.base_station_current = 0.0f;
    lora.base_station_soc = 0.0f;
    lora.rssi = 0.0f;
    lora.snr = 0.0f;

    sensor_converter.packLoRaData(lora, out_payload);
    return true;
}

static void serviceLoRa()
{
    if (!config::USE_LORA_RADIO)
    {
        return;
    }

    lora_comms.service();

    const uint32_t now_ms = millis();
    const uint32_t period_ms = (config::LORA_TX_RATE_HZ > 0)
        ? (1000U / config::LORA_TX_RATE_HZ)
        : 40U;
    if ((now_ms - last_lora_tx_ms) < period_ms)
    {
        return;
    }
    if (!lora_comms.canSend())
    {
        return;
    }

    uint8_t payload[SIZE_OF_LORA_DATA] = {0};
    if (!buildLoRaPayload(payload))
    {
        return;
    }
    last_lora_tx_ms = now_ms;
    lora_in_rx_mode = false;  // Exiting RX for TX
    if (lora_comms.send(payload, sizeof(payload)))
    {
        lora_tx_ok++;
    }
    else
    {
        lora_tx_fail++;
    }
}

// ============================================================================
// Config readback: send current config to app over BLE
// ============================================================================

static void sendCurrentConfig()
{
    String j = "{\"type\":\"config\"";
    j += ",\"sb1\":"; j += cfg_servo_bias1;
    j += ",\"shz\":"; j += cfg_servo_hz;
    j += ",\"smn\":"; j += cfg_servo_min;
    j += ",\"smx\":"; j += cfg_servo_max;
    j += ",\"kp\":";  j += String(cfg_pid_kp, 4);
    j += ",\"ki\":";  j += String(cfg_pid_ki, 4);
    j += ",\"kd\":";  j += String(cfg_pid_kd, 4);
    j += ",\"pmn\":"; j += String(cfg_pid_min, 1);
    j += ",\"pmx\":"; j += String(cfg_pid_max, 1);
    j += ",\"sen\":"; j += cfg_servo_enabled ? "true" : "false";
    j += ",\"gs\":";  j += cfg_gain_sched ? "true" : "false";
    j += ",\"ac\":";  j += cfg_use_angle_ctrl ? "true" : "false";
    j += ",\"rdly\":"; j += cfg_roll_delay_ms;
    // LoRa settings — so app can verify actual device config
    j += ",\"lf\":";  j += String(lora_freq_mhz, 1);
    j += ",\"lsf\":"; j += lora_sf;
    j += ",\"lbw\":"; j += String(lora_bw_khz, 0);
    j += ",\"lcr\":"; j += lora_cr;
    j += ",\"lpw\":"; j += lora_tx_power;
    j += "}";
    ble_app.sendConfigJSON(j);
    Serial.println("[CFG] Sent config readback to app");
}

// Cache servo config to NVS (mirrors what FlightComputer stores)
static void cacheServoConfig(const uint8_t* payload, size_t len)
{
    if (len < 14) return;
    ServoConfigData sc;
    memcpy(&sc, payload, sizeof(sc));
    cfg_servo_bias1 = sc.bias_us[0];
    cfg_servo_hz    = sc.hz;
    cfg_servo_min   = sc.min_us;
    cfg_servo_max   = sc.max_us;
    prefs.begin("servo", false);
    prefs.putShort("b1",  sc.bias_us[0]);
    prefs.putShort("hz",  sc.hz);
    prefs.putShort("min", sc.min_us);
    prefs.putShort("max", sc.max_us);
    prefs.end();
    Serial.printf("[CFG] Servo config cached: bias=%d hz=%d min=%d max=%d\n",
        sc.bias_us[0], sc.hz, sc.min_us, sc.max_us);
}

// Cache PID config to NVS
static void cachePIDConfig(const uint8_t* payload, size_t len)
{
    if (len < 20) return;
    PIDConfigData pc;
    memcpy(&pc, payload, sizeof(pc));
    cfg_pid_kp = pc.kp; cfg_pid_ki = pc.ki; cfg_pid_kd = pc.kd;
    cfg_pid_min = pc.min_cmd; cfg_pid_max = pc.max_cmd;
    prefs.begin("pid", false);
    prefs.putFloat("kp", pc.kp);
    prefs.putFloat("ki", pc.ki);
    prefs.putFloat("kd", pc.kd);
    prefs.putFloat("mn", pc.min_cmd);
    prefs.putFloat("mx", pc.max_cmd);
    prefs.end();
    Serial.printf("[CFG] PID config cached: Kp=%.4f Ki=%.4f Kd=%.4f [%.1f,%.1f]\n",
        pc.kp, pc.ki, pc.kd, pc.min_cmd, pc.max_cmd);
}

// Cache roll control config to NVS
static void cacheRollControlConfig(const uint8_t* payload, size_t len)
{
    if (len < 4) return;
    RollControlConfigData rc;
    memcpy(&rc, payload, sizeof(rc));
    cfg_use_angle_ctrl = (rc.use_angle_control != 0);
    cfg_roll_delay_ms  = rc.roll_delay_ms;
    prefs.begin("roll", false);
    prefs.putBool("ac", cfg_use_angle_ctrl);
    prefs.putUShort("rdly", cfg_roll_delay_ms);
    prefs.end();
    Serial.printf("[CFG] Roll control cached: angle_ctrl=%s delay=%u ms\n",
        cfg_use_angle_ctrl ? "ON" : "OFF", (unsigned)cfg_roll_delay_ms);
}

// ============================================================================
// LoRa Uplink RX (receive sim commands from BaseStation)
// ============================================================================

static void processUplinkCommand(uint8_t cmd, const uint8_t* payload, size_t payload_len)
{
    Serial.printf("[UPLINK RX] cmd=%u payload_len=%u\n", cmd, (unsigned)payload_len);

    if (cmd == 1)
    {
        // Camera: payload[0] = desired state (1 = on, 0 = off).
        // Payload makes retries idempotent (won't toggle back and forth).
        // Falls back to toggle if no payload (legacy compat).
        bool want_on = (payload_len >= 1) ? (payload[0] != 0)
                                          : !camera_recording_requested;
        if (want_on != camera_recording_requested)
        {
            camera_recording_requested = want_on;
            setPendingCommand(want_on ? CAMERA_START : CAMERA_STOP);
            Serial.printf("[UPLINK] Camera %s\n", want_on ? "START" : "STOP");
        }
        else
        {
            Serial.printf("[UPLINK] Camera already %s, ignoring\n",
                          want_on ? "ON" : "OFF");
        }
    }
    else if (cmd == 23)
    {
        // Logging: payload[0] = desired state (1 = start, 0 = stop).
        // Payload makes retries idempotent (won't toggle back and forth).
        // Falls back to toggle if no payload (legacy compat).
        bool want_on = (payload_len >= 1) ? (payload[0] != 0)
                                          : !logger.isLoggingActive();
        if (want_on && !logger.isLoggingActive())
        {
            logger.startLogging();
            Serial.println("[UPLINK] Logging started");
        }
        else if (!want_on && logger.isLoggingActive())
        {
            logger.endLogging();
            Serial.println("[UPLINK] Logging stopped");
        }
        else
        {
            Serial.printf("[UPLINK] Logging already %s, ignoring\n",
                          want_on ? "ON" : "OFF");
        }
    }
    else if (cmd == 24 && payload_len >= sizeof(ServoTestAnglesData))
    {
        // Servo test angles: relay to FlightComputer via I2C
        memcpy(pending_config_data, payload, sizeof(ServoTestAnglesData));
        pending_config_data_len = sizeof(ServoTestAnglesData);
        pending_config_msg_type = SERVO_TEST_MSG;
        setPendingCommand(SERVO_TEST_PENDING);
        Serial.println("[UPLINK] Servo test angles queued");
    }
    else if (cmd == 25)
    {
        // Servo test stop
        setPendingCommand(SERVO_TEST_STOP);
        Serial.println("[UPLINK] Servo test stop");
    }
    else if (cmd == 5 && payload_len >= 12)
    {
        // Sim config: relay to FlightComputer via I2C
        SimConfigData sim_cfg;
        float mass_g;
        memcpy(&mass_g,              payload + 0, 4);
        memcpy(&sim_cfg.thrust_n,    payload + 4, 4);
        memcpy(&sim_cfg.burn_time_s, payload + 8, 4);
        sim_cfg.mass_kg = mass_g / 1000.0f;
        sim_cfg.descent_rate_mps = 0.0f;
        if (payload_len >= 16) {
            memcpy(&sim_cfg.descent_rate_mps, payload + 12, 4);
        }
        memcpy(pending_config_data, &sim_cfg, sizeof(sim_cfg));
        pending_config_data_len = sizeof(sim_cfg);
        pending_config_msg_type = SIM_CONFIG_MSG;
        setPendingCommand(SIM_CONFIG_PENDING);
        Serial.printf("[UPLINK] Sim config queued: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s\n",
                      (double)mass_g, (double)sim_cfg.thrust_n,
                      (double)sim_cfg.burn_time_s, (double)sim_cfg.descent_rate_mps);
    }
    else if (cmd == 6)
    {
        setPendingCommand(SIM_START_CMD);
        Serial.println("[UPLINK] Sim start queued for FlightComputer");
    }
    else if (cmd == 7)
    {
        logger.endLogging();
        setPendingCommand(SIM_STOP_CMD);
        Serial.println("[UPLINK] Sim stop queued for FlightComputer (logging ended)");
    }
    else if (cmd == 10 && payload_len >= 11)
    {
        // LoRa reconfiguration via uplink: [freq:4f][bw:4f][sf:1][cr:1][txpwr:1]
        float new_freq, new_bw;
        memcpy(&new_freq, payload + 0, 4);
        memcpy(&new_bw,   payload + 4, 4);
        uint8_t new_sf   = payload[8];
        uint8_t new_cr   = payload[9];
        int8_t  new_pwr  = (int8_t)payload[10];

        if (lora_comms.reconfigure(new_freq, new_sf, new_bw, new_cr, new_pwr))
        {
            lora_freq_mhz = new_freq;
            lora_bw_khz   = new_bw;
            lora_sf        = new_sf;
            lora_cr        = new_cr;
            lora_tx_power  = new_pwr;

            prefs.begin("lora", false);
            prefs.putFloat("freq",  lora_freq_mhz);
            prefs.putFloat("bw",    lora_bw_khz);
            prefs.putUChar("sf",    lora_sf);
            prefs.putUChar("cr",    lora_cr);
            prefs.putChar("txpwr",  lora_tx_power);
            prefs.end();

            Serial.printf("[UPLINK] LoRa reconfigured + saved: %.1f MHz SF%u BW%.0f CR%u %d dBm\n",
                          (double)lora_freq_mhz, (unsigned)lora_sf,
                          (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);
        }
        else
        {
            Serial.println("[UPLINK] LoRa reconfigure FAILED");
        }
    }
    else if (cmd == 12 && payload_len >= 14)
    {
        // Servo config from BaseStation: relay to FlightComputer + cache
        memcpy(pending_config_data, payload, 14);
        pending_config_data_len = 14;
        pending_config_msg_type = SERVO_CONFIG_MSG;
        setPendingCommand(SERVO_CONFIG_PENDING);
        cacheServoConfig(payload, payload_len);
        Serial.println("[UPLINK] Servo config queued for RocketComputer");
    }
    else if (cmd == 13 && payload_len >= 20)
    {
        // PID config from BaseStation: relay to FlightComputer + cache
        memcpy(pending_config_data, payload, 20);
        pending_config_data_len = 20;
        pending_config_msg_type = PID_CONFIG_MSG;
        setPendingCommand(PID_CONFIG_PENDING);
        cachePIDConfig(payload, payload_len);
        Serial.println("[UPLINK] PID config queued for RocketComputer");
    }
    else if (cmd == 14 && payload_len >= 1)
    {
        // Servo control enable/disable from BaseStation
        bool enabled = (payload[0] != 0);
        cfg_servo_enabled = enabled;
        setPendingCommand(enabled ? SERVO_CTRL_ENABLE : SERVO_CTRL_DISABLE);
        Serial.printf("[UPLINK] Servo control: %s\n", enabled ? "ENABLE" : "DISABLE");
    }
    else if (cmd == 22 && payload_len >= 1)
    {
        // Gain schedule enable/disable from BaseStation
        bool enabled = (payload[0] != 0);
        cfg_gain_sched = enabled;
        setPendingCommand(enabled ? GAIN_SCHED_ENABLE : GAIN_SCHED_DISABLE);
        Serial.printf("[UPLINK] Gain schedule: %s\n", enabled ? "ENABLE" : "DISABLE");
    }
    else if (cmd == 31 && payload_len >= 4)
    {
        // Roll control config from BaseStation
        memcpy(pending_config_data, payload, 4);
        pending_config_data_len = 4;
        pending_config_msg_type = ROLL_CTRL_CONFIG_MSG;
        setPendingCommand(ROLL_CTRL_CONFIG_PENDING);
        cacheRollControlConfig(payload, payload_len);
        Serial.println("[UPLINK] Roll control config queued for RocketComputer");
    }
    else
    {
        Serial.printf("[UPLINK] Unknown cmd %u\n", cmd);
    }
}

/// Enter RX mode between TX cycles and check for uplink commands
static void serviceLoRaUplink()
{
    if (!config::USE_LORA_RADIO) return;

    lora_comms.service();  // Complete any pending TX (auto-enters RX after TX)

    // Only enter RX when radio is idle (not transmitting)
    if (!lora_comms.canSend()) return;

    // service() auto-calls startReceive() after TX completion.
    // Sync our tracking flag to avoid a redundant startReceive() that
    // would reset rx_done_ and potentially drop a received packet.
    if (lora_comms.isInRxMode())
    {
        lora_in_rx_mode = true;
    }

    // Enter RX mode if not already (first call before any TX has occurred)
    if (!lora_in_rx_mode)
    {
        if (!lora_comms.startReceive()) return;
        lora_in_rx_mode = true;
    }

    // Poll DIO1 pin directly as fallback for hardware interrupt.
    // The ISR fires reliably for TX-done events, but on some ESP32-S3
    // boards the interrupt may not trigger for RX-done.  pollDio1()
    // checks the pin level and sets rx_done_ if DIO1 is asserted.
    lora_comms.pollDio1();

    // Non-blocking poll for uplink packet
    uint8_t rx_buf[32];
    size_t rx_len = 0;

    if (lora_comms.readPacket(rx_buf, sizeof(rx_buf), rx_len))
    {
        if (rx_len >= 3 && rx_buf[0] == config::UPLINK_SYNC_BYTE)
        {
            uint8_t cmd = rx_buf[1];
            uint8_t payload_len = rx_buf[2];
            if (rx_len >= (size_t)(3 + payload_len))
            {
                processUplinkCommand(cmd, &rx_buf[3], payload_len);
                lora_uplink_rx_count++;
            }
        }
        // readPacket() internally re-enters RX mode after reading
    }
}

static void printLoRaPayloadDebug()
{
    if (!config::USE_LORA_RADIO)
    {
        return;
    }

    uint8_t payload[SIZE_OF_LORA_DATA] = {0};
    if (!buildLoRaPayload(payload))
    {
        return;
    }

    LoRaDataSI decoded = {};
    sensor_converter.unpackLoRa(payload, decoded);
    Serial.printf("[OUT] LoRa tx sats/pdop=%u/%.1f | ecef(m)=%.0f,%.0f,%.0f | alt/rate/max/mspd=%.1f/%.1f/%.1f/%.1f\n",
                  (unsigned)decoded.num_sats,
                  (double)decoded.pdop,
                  (double)decoded.ecef_x,
                  (double)decoded.ecef_y,
                  (double)decoded.ecef_z,
                  (double)decoded.pressure_alt,
                  (double)decoded.altitude_rate,
                  (double)decoded.max_alt,
                  (double)decoded.max_speed);
    Serial.printf("[OUT] LoRa tx state/flags=%u/%u%u%u%u | acc=%.1f,%.1f,%.1f | gyro=%.1f,%.1f,%.1f | v/i/soc=%.2f/%.0f/%.0f\n",
                  (unsigned)decoded.rocket_state,
                  decoded.launch_flag ? 1U : 0U,
                  decoded.vel_u_apogee_flag ? 1U : 0U,
                  decoded.alt_apogee_flag ? 1U : 0U,
                  decoded.alt_landed_flag ? 1U : 0U,
                  (double)decoded.acc_x,
                  (double)decoded.acc_y,
                  (double)decoded.acc_z,
                  (double)decoded.gyro_x,
                  (double)decoded.gyro_y,
                  (double)decoded.gyro_z,
                  (double)decoded.voltage,
                  (double)decoded.current,
                  (double)decoded.soc);
}

static void printStats()
{
    const uint32_t now = millis();
    if ((now - last_stats_ms) < config::STATS_PERIOD_MS)
    {
        return;
    }
    const uint32_t dt = now - last_stats_ms;
    last_stats_ms = now;

    // --- Low-power mode: send minimal BLE telemetry only ---
    if (!pwr_pin_on)
    {
        TR_BLE_To_APP::TelemetryData ble_telem = {};
        // TODO: Read battery from INA230 when hardware is ready
        ble_telem.soc = NAN;
        ble_telem.current = NAN;
        ble_telem.voltage = NAN;
        ble_telem.latitude = NAN;
        ble_telem.longitude = NAN;
        ble_telem.gdop = NAN;
        ble_telem.num_sats = 0;
        ble_telem.state = "OFF";
        ble_telem.camera_recording = false;
        ble_telem.logging_active = false;
        ble_telem.active_file = "";
        ble_telem.rx_kbs = NAN;
        ble_telem.wr_kbs = NAN;
        ble_telem.frames_rx = 0;
        ble_telem.frames_drop = 0;
        ble_telem.max_alt_m = NAN;
        ble_telem.max_speed_mps = NAN;
        ble_telem.pressure_alt = NAN;
        ble_telem.altitude_rate = NAN;
        ble_telem.rssi = NAN;
        ble_telem.snr = NAN;
        ble_telem.roll = NAN;
        ble_telem.pitch = NAN;
        ble_telem.yaw = NAN;
        ble_telem.roll_cmd = NAN;
        ble_telem.bs_soc = NAN;
        ble_telem.bs_voltage = NAN;
        ble_telem.bs_current = NAN;
        ble_telem.pwr_pin_on = false;
        ble_app.sendTelemetry(ble_telem);
        return;
    }

    // --- Active mode: full stats and telemetry ---
    TR_LogToFlashStats s = {};
    logger.getStats(s);

    // Capture GNSS timestamp for the active log file (when available)
    if (s.logging_active && latest_gnss_valid && latest_gnss_si.year > 2000)
    {
        uint16_t ts_year   = latest_gnss_si.year;
        uint8_t  ts_month  = latest_gnss_si.month;
        uint8_t  ts_day    = latest_gnss_si.day;
        uint8_t  ts_hour   = latest_gnss_si.hour;
        uint8_t  ts_minute = latest_gnss_si.minute;
        uint8_t  ts_second = latest_gnss_si.second;

        // Sim mode uses hardcoded GNSS date (2025-01-01 12:00:xx).
        // Substitute phone-synced time for unique filenames.
        if (phone_time_valid &&
            ts_year == 2025 && ts_month == 1 && ts_day == 1 &&
            ts_hour == 12 && ts_minute == 0)
        {
            uint32_t elapsed_s = (millis() - phone_sync_millis) / 1000;
            uint32_t total_s = (uint32_t)phone_utc_hour * 3600U +
                               (uint32_t)phone_utc_minute * 60U +
                               (uint32_t)phone_utc_second + elapsed_s;
            ts_year   = phone_utc_year;
            ts_month  = phone_utc_month;
            ts_day    = phone_utc_day;
            if (total_s >= 86400U)
            {
                ts_day += (uint8_t)(total_s / 86400U);
                total_s %= 86400U;
            }
            ts_hour   = (uint8_t)(total_s / 3600U);
            ts_minute = (uint8_t)((total_s % 3600U) / 60U);
            ts_second = (uint8_t)(total_s % 60U);
        }

        logger.setFileTimestamp(logger.currentFilename(),
                               ts_year, ts_month, ts_day,
                               ts_hour, ts_minute, ts_second);
    }

    if (s.ring_fill > interval_ring_fill_peak)
    {
        interval_ring_fill_peak = s.ring_fill;
    }
    const uint64_t rx_delta = s.bytes_received - prev_bytes_rx;
    const uint64_t nand_delta = s.bytes_written_nand - prev_bytes_nand;
    const uint64_t raw_i2c_delta = raw_i2c_bytes - prev_raw_i2c_bytes;
    const uint32_t ring_overrun_delta = s.ring_overruns - prev_ring_overruns;
    const uint32_t ring_drop_oldest_delta = s.ring_drop_oldest_bytes - prev_ring_drop_oldest_bytes;
    prev_bytes_rx = s.bytes_received;
    prev_bytes_nand = s.bytes_written_nand;
    prev_raw_i2c_bytes = raw_i2c_bytes;
    prev_ring_overruns = s.ring_overruns;
    prev_ring_drop_oldest_bytes = s.ring_drop_oldest_bytes;

    const float rx_kbs = (dt > 0) ? ((float)rx_delta / (float)dt) : 0.0f;
    const float wr_kbs = (dt > 0) ? ((float)nand_delta / (float)dt) : 0.0f;
    const float raw_rx_kbs = (dt > 0) ? ((float)raw_i2c_delta / (float)dt) : 0.0f;
    const uint32_t d_query = msg_count_query - prev_msg_count_query;
    const uint32_t d_ism6 = msg_count_ism6 - prev_msg_count_ism6;
    const uint32_t d_bmp = msg_count_bmp - prev_msg_count_bmp;
    const uint32_t d_mmc = msg_count_mmc - prev_msg_count_mmc;
    const uint32_t d_gnss = msg_count_gnss - prev_msg_count_gnss;
    const uint32_t d_non_sensor = msg_count_non_sensor - prev_msg_count_non_sensor;
    const uint32_t d_power = msg_count_power - prev_msg_count_power;
    const uint32_t d_start_logging = msg_count_start_logging - prev_msg_count_start_logging;
    const uint32_t d_end_flight = msg_count_end_flight - prev_msg_count_end_flight;
    const uint32_t d_unknown = msg_count_unknown - prev_msg_count_unknown;
    const float hz_scale = (dt > 0U) ? (1000.0f / (float)dt) : 0.0f;
    const float hz_query = (float)d_query * hz_scale;
    const float hz_ism6 = (float)d_ism6 * hz_scale;
    const float hz_bmp = (float)d_bmp * hz_scale;
    const float hz_mmc = (float)d_mmc * hz_scale;
    const float hz_gnss = (float)d_gnss * hz_scale;
    const float hz_non_sensor = (float)d_non_sensor * hz_scale;
    const float hz_power = (float)d_power * hz_scale;
    const float hz_start_logging = (float)d_start_logging * hz_scale;
    const float hz_end_flight = (float)d_end_flight * hz_scale;
    const float hz_unknown = (float)d_unknown * hz_scale;
    prev_msg_count_query = msg_count_query;
    prev_msg_count_ism6 = msg_count_ism6;
    prev_msg_count_bmp = msg_count_bmp;
    prev_msg_count_mmc = msg_count_mmc;
    prev_msg_count_gnss = msg_count_gnss;
    prev_msg_count_non_sensor = msg_count_non_sensor;
    prev_msg_count_power = msg_count_power;
    prev_msg_count_start_logging = msg_count_start_logging;
    prev_msg_count_end_flight = msg_count_end_flight;
    prev_msg_count_unknown = msg_count_unknown;

    if (config::VERBOSE_DEBUG)
    {
    Serial.printf("[OUT] RX %.1f KB/s | WR %.1f KB/s | frames rx/drop/bad=%lu/%lu/%lu | RING=%lu/%lu (hi=%lu)\n",
                  (double)rx_kbs,
                  (double)wr_kbs,
                  (unsigned long)s.frames_received,
                  (unsigned long)s.frames_dropped,
                  (unsigned long)frames_bad_crc,
                  (unsigned long)s.ring_fill,
                  (unsigned long)config::RAM_RING_SIZE,
                  (unsigned long)s.ring_highwater);
    Serial.printf("[OUT] RING interval peak/overrun/drop_oldest_bytes=%lu/%lu/%lu\n",
                  (unsigned long)interval_ring_fill_peak,
                  (unsigned long)ring_overrun_delta,
                  (unsigned long)ring_drop_oldest_delta);
    Serial.printf("[OUT] i2c raw reads/bytes=%lu/%llu\n",
                  (unsigned long)raw_i2c_reads,
                  (unsigned long long)raw_i2c_bytes);
    Serial.printf("[OUT] i2c raw_rx=%.1f KB/s | ring_drops=%lu | parser_drops resync/len/crc=%llu/%llu/%lu\n",
                  (double)raw_rx_kbs,
                  (unsigned long)rx_ring_overflow_drops,
                  (unsigned long long)parser_resync_drops,
                  (unsigned long long)parser_len_drops,
                  (unsigned long)frames_bad_crc);
    Serial.printf("[OUT] logging=%c file=%s page=%lu block=%lu prog_fail=%lu erase_fail=%lu\n",
                  s.logging_active ? 'Y' : 'N',
                  logger.currentFilename(),
                  (unsigned long)s.nand_page,
                  (unsigned long)s.nand_block,
                  (unsigned long)s.nand_prog_fail,
                  (unsigned long)s.nand_erase_fail);
    Serial.printf("[OUT] cfg fs=%u/%u/%u rot_z(ism/mmc)=%.2f/%.2f deg v%u\n",
                  (unsigned)last_query_cfg.ism6_low_g_fs_g,
                  (unsigned)last_query_cfg.ism6_high_g_fs_g,
                  (unsigned)last_query_cfg.ism6_gyro_fs_dps,
                  (double)((float)last_query_cfg.ism6_rot_z_cdeg / 100.0f),
                  (double)((float)last_query_cfg.mmc_rot_z_cdeg / 100.0f),
                  (unsigned)last_query_cfg.format_version);
    Serial.printf("[OUT] msg Hz q/ism6/bmp/mmc/gnss/ns/pwr/st/en/unk=%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f\n",
                  (double)hz_query,
                  (double)hz_ism6,
                  (double)hz_bmp,
                  (double)hz_mmc,
                  (double)hz_gnss,
                  (double)hz_non_sensor,
                  (double)hz_power,
                  (double)hz_start_logging,
                  (double)hz_end_flight,
                  (double)hz_unknown);
    if (config::USE_LORA_RADIO)
    {
        TR_LoRa_Comms::Stats ls = {};
        lora_comms.getStats(ls);
        Serial.printf("[OUT] LoRa en=%c tx_start/ok/fail=%lu/%lu/%lu local_ok/fail=%lu/%lu last_err=%d tx=%c\n",
                      ls.enabled ? 'Y' : 'N',
                      (unsigned long)ls.tx_started,
                      (unsigned long)ls.tx_ok,
                      (unsigned long)ls.tx_fail,
                      (unsigned long)lora_tx_ok,
                      (unsigned long)lora_tx_fail,
                      (int)ls.last_error,
                      ls.transmitting ? 'Y' : 'N');
        printLoRaPayloadDebug();
    }
    } // End VERBOSE_DEBUG

    // Always print compact LoRa uplink stats (helps diagnose uplink issues)
    if (config::USE_LORA_RADIO)
    {
        TR_LoRa_Comms::Stats ls = {};
        lora_comms.getStats(ls);
        Serial.printf("[OUT] LoRa tx=%lu/%lu rx=%lu crc_fail=%lu isr=%lu uplink_rx=%lu rxmode=%c\n",
                      (unsigned long)ls.tx_ok,
                      (unsigned long)ls.tx_fail,
                      (unsigned long)ls.rx_count,
                      (unsigned long)ls.rx_crc_fail,
                      (unsigned long)ls.isr_count,
                      (unsigned long)lora_uplink_rx_count,
                      ls.rx_mode ? 'Y' : 'N');
    }

    // Send telemetry to BLE app
    TR_BLE_To_APP::TelemetryData ble_telem = {};
    ble_telem.soc = NAN;
    ble_telem.current = NAN;
    ble_telem.voltage = NAN;
    ble_telem.latitude = NAN;
    ble_telem.longitude = NAN;
    ble_telem.gdop = NAN;
    ble_telem.num_sats = 0;
    if (latest_power_valid)
    {
        POWERDataSI p = {};
        sensor_converter.convertPowerData(latest_power_raw, p);
        ble_telem.soc = p.soc;
        ble_telem.current = p.current;
        ble_telem.voltage = p.voltage;
    }
    if (latest_gnss_valid)
    {
        ble_telem.latitude = latest_gnss_si.lat;
        ble_telem.longitude = latest_gnss_si.lon;
        ble_telem.gdop = latest_gnss_si.pdop;
        ble_telem.num_sats = (int)latest_gnss_si.num_sats;
    }
    ble_telem.state = rocketStateToString(latest_rocket_state);
    ble_telem.camera_recording = camera_recording_requested;
    ble_telem.logging_active = s.logging_active;
    ble_telem.active_file = logger.currentFilename();
    ble_telem.rx_kbs = rx_kbs;
    ble_telem.wr_kbs = wr_kbs;
    ble_telem.frames_rx = s.frames_received;
    ble_telem.frames_drop = s.frames_dropped;
    ble_telem.max_alt_m = max_alt_m;
    ble_telem.max_speed_mps = max_speed_mps;
    ble_telem.pressure_alt = pressure_alt_m;
    ble_telem.altitude_rate = pressure_alt_rate_mps;
    if (latest_ism6_valid)
    {
        ISM6HG256DataSI ism_si = {};
        sensor_converter.convertISM6HG256Data(latest_ism6_raw, ism_si);
        ble_telem.low_g_x = ism_si.low_g_acc_x;
        ble_telem.low_g_y = ism_si.low_g_acc_y;
        ble_telem.low_g_z = ism_si.low_g_acc_z;
        ble_telem.high_g_x = ism_si.high_g_acc_x;
        ble_telem.high_g_y = ism_si.high_g_acc_y;
        ble_telem.high_g_z = ism_si.high_g_acc_z;
        ble_telem.gyro_x = ism_si.gyro_x;
        ble_telem.gyro_y = ism_si.gyro_y;
        ble_telem.gyro_z = ism_si.gyro_z;
    }
    // Attitude from FlightComputer (quaternion → Euler degrees)
    ble_telem.roll     = ns_roll_deg;
    ble_telem.pitch    = ns_pitch_deg;
    ble_telem.yaw      = ns_yaw_deg;
    ble_telem.roll_cmd = (float)latest_non_sensor.roll_cmd / 100.0f;
    ble_telem.rssi = NAN;  // LoRa RSSI only meaningful on base station (continuous RX)
    ble_telem.snr = NAN;
    ble_telem.bs_soc = NAN;      // No base station battery
    ble_telem.bs_voltage = NAN;
    ble_telem.bs_current = NAN;
    // Flight event flags
    ble_telem.launch_flag       = nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
    ble_telem.vel_u_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
    ble_telem.alt_apogee_flag   = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
    ble_telem.alt_landed_flag   = nsFlagSet(latest_non_sensor.flags, NSF_ALT_LANDED);
    ble_telem.pwr_pin_on        = pwr_pin_on;
    ble_app.sendTelemetry(ble_telem);

    interval_ring_fill_peak = s.ring_fill;
}

// Initialize peripherals that are behind the PWR_PIN power rail.
// Called once when power is first turned on (deferred from setup).
void initPeripherals()
{
    if (peripherals_initialized) return;

    Serial.println("[PWR] Initializing peripherals...");

    SPI.begin(config::SPI_SCK, config::SPI_MISO, config::SPI_MOSI);
    delay(20);

    TR_LogToFlashConfig log_cfg = {};
    log_cfg.nand_cs = config::NAND_CS;
    log_cfg.spi_hz_nand = config::SPI_HZ_NAND;
    log_cfg.spi_mode_nand = config::SPI_MODE_NAND;
    log_cfg.ring_buffer_size = config::RAM_RING_SIZE;
    log_cfg.debug = config::DEBUG;
    // MRAM ring buffer (128 KB on shared SPI bus — replaces 64 KB RAM ring)
    log_cfg.mram_cs = config::MRAM_CS;
    log_cfg.spi_hz_mram = config::SPI_HZ_MRAM;
    log_cfg.spi_mode_mram = config::SPI_MODE_MRAM;
    log_cfg.mram_size = config::MRAM_SIZE;
    if (!logger.begin(SPI, log_cfg))
    {
        Serial.println("[PWR] TR_LogToFlash begin failed");
        return;
    }
    // Start the NAND flush task on Core 0 — decouples LittleFS writes from
    // the main loop so the RAM ring can buffer during NAND stalls.
    logger.startFlushTask(/* core */ 0, /* stackSize */ 8192, /* priority */ 1);

    TR_LogToFlashRecoveryInfo recovery = {};
    logger.getRecoveryInfo(recovery);
    if (recovery.recovered)
    {
        Serial.printf("[LOG] Startup recovery wrote %lu bytes to %s\n",
                      (unsigned long)recovery.recovered_bytes,
                      recovery.filename);
    }

    if (config::USE_LORA_RADIO)
    {
        // Load saved LoRa config from NVS (write config.h defaults if empty)
        prefs.begin("lora", false);  // read-write
        if (!prefs.isKey("freq"))
        {
            // First boot or NVS erased — seed with config.h factory defaults
            prefs.putFloat("freq",  config::LORA_FREQ_MHZ);
            prefs.putUChar("sf",    config::LORA_SF);
            prefs.putFloat("bw",    config::LORA_BW_KHZ);
            prefs.putUChar("cr",    config::LORA_CR);
            prefs.putChar("txpwr",  config::LORA_TX_POWER_DBM);
            Serial.println("[CFG] LoRa NVS empty — wrote config.h defaults");
        }
        lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
        lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
        lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
        lora_cr        = prefs.getUChar("cr",   config::LORA_CR);
        lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
        prefs.end();
        Serial.printf("[CFG] LoRa NVS: %.1f MHz SF%u BW%.0f CR%u %d dBm\n",
                      (double)lora_freq_mhz, (unsigned)lora_sf,
                      (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

        // Load cached servo config from NVS
        prefs.begin("servo", true);
        cfg_servo_bias1 = prefs.getShort("b1",  cfg_servo_bias1);
        cfg_servo_hz    = prefs.getShort("hz",  cfg_servo_hz);
        cfg_servo_min   = prefs.getShort("min", cfg_servo_min);
        cfg_servo_max   = prefs.getShort("max", cfg_servo_max);
        prefs.end();
        Serial.printf("[NVS] Servo cache: bias=%d hz=%d min=%d max=%d\n",
            cfg_servo_bias1, cfg_servo_hz, cfg_servo_min, cfg_servo_max);

        // Load cached PID config from NVS
        prefs.begin("pid", true);
        cfg_pid_kp  = prefs.getFloat("kp",  cfg_pid_kp);
        cfg_pid_ki  = prefs.getFloat("ki",  cfg_pid_ki);
        cfg_pid_kd  = prefs.getFloat("kd",  cfg_pid_kd);
        cfg_pid_min = prefs.getFloat("mn",  cfg_pid_min);
        cfg_pid_max = prefs.getFloat("mx",  cfg_pid_max);
        cfg_gain_sched = prefs.getBool("gs", cfg_gain_sched);
        prefs.end();
        Serial.printf("[NVS] PID cache: Kp=%.4f Ki=%.4f Kd=%.4f [%.1f,%.1f] GS=%s\n",
            cfg_pid_kp, cfg_pid_ki, cfg_pid_kd, cfg_pid_min, cfg_pid_max,
            cfg_gain_sched ? "ON" : "OFF");

        // Load cached roll control config from NVS
        prefs.begin("roll", true);
        cfg_use_angle_ctrl = prefs.getBool("ac", cfg_use_angle_ctrl);
        cfg_roll_delay_ms  = prefs.getUShort("rdly", cfg_roll_delay_ms);
        prefs.end();
        Serial.printf("[NVS] Roll control: angle_ctrl=%s delay=%u ms\n",
            cfg_use_angle_ctrl ? "ON" : "OFF", (unsigned)cfg_roll_delay_ms);

        lora_spi.begin(config::LORA_SPI_SCK,
                       config::LORA_SPI_MISO,
                       config::LORA_SPI_MOSI,
                       config::LORA_CS_PIN);

        TR_LoRa_Comms::Config lora_cfg = {};
        lora_cfg.enabled = config::USE_LORA_RADIO;
        lora_cfg.cs_pin = config::LORA_CS_PIN;
        lora_cfg.dio1_pin = config::LORA_DIO1_PIN;
        lora_cfg.rst_pin = config::LORA_RST_PIN;
        lora_cfg.busy_pin = config::LORA_BUSY_PIN;
        lora_cfg.freq_mhz = lora_freq_mhz;
        lora_cfg.spreading_factor = lora_sf;
        lora_cfg.bandwidth_khz = lora_bw_khz;
        lora_cfg.coding_rate = lora_cr;
        lora_cfg.preamble_len = config::LORA_PREAMBLE_LEN;
        lora_cfg.tx_power_dbm = lora_tx_power;
        lora_cfg.crc_on = config::LORA_CRC_ON;
        lora_cfg.rx_boosted_gain = config::LORA_RX_BOOSTED_GAIN;
        lora_cfg.syncword_private = config::LORA_SYNCWORD_PRIVATE;
        if (!lora_comms.begin(lora_spi, lora_cfg, config::DEBUG))
        {
            Serial.println("[PWR] LoRa init failed");
        }
    }

    const esp_err_t i2c_ok = i2c_interface.beginSlave(config::I2C_SDA_PIN,
                                                      config::I2C_SCL_PIN,
                                                      config::I2C_CLOCK_HZ,
                                                      config::I2C_SLAVE_RX_BUF,
                                                      config::I2C_SLAVE_TX_BUF,
                                                      false,
                                                      false);
    if (i2c_ok != ESP_OK)
    {
        Serial.printf("[PWR] I2C slave init failed: %d\n", (int)i2c_ok);
    }
    else
    {
        Serial.printf("[PWR] I2C slave on port=%d addr=0x%02X SDA=%d SCL=%d clk=%lu\n",
                      (int)config::I2C_PORT,
                      (unsigned)config::I2C_ADDRESS,
                      (int)config::I2C_SDA_PIN,
                      (int)config::I2C_SCL_PIN,
                      (unsigned long)config::I2C_CLOCK_HZ);
    }

    queueOutStatusResponse(true);

    peripherals_initialized = true;
    Serial.println("[PWR] Peripherals initialized.");
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(config::PWR_PIN, OUTPUT);
    digitalWrite(config::PWR_PIN, LOW);   // Start with power rail OFF
    pwr_pin_on = false;

    Serial.println("Starting OutComputer (low-power mode)...");

    // Only BLE starts at boot — everything else is behind PWR_PIN
    if (!ble_app.begin())
    {
        Serial.println("BLE app interface failed to start");
    }

    last_stats_ms = millis();
    Serial.println("OutComputer ready (PWR_PIN OFF, waiting for power-on command).");
}

void loop()
{
    // Simple test commands via Serial Monitor (only when peripherals are active)
    if (pwr_pin_on && peripherals_initialized && Serial.available())
    {
        char cmd = Serial.read();
        if (cmd == 's' || cmd == 'S')
        {
            Serial.println("[TEST] Starting logging...");
            logger.startLogging();
        }
        else if (cmd == 'e' || cmd == 'E')
        {
            Serial.println("[TEST] Ending logging...");
            logger.endLogging();
        }
        else if (cmd == 'w' || cmd == 'W')
        {
            // Write some test data
            uint8_t test_frame[64];
            for (int i = 0; i < 64; i++) {
                test_frame[i] = i;
            }
            if (logger.enqueueFrame(test_frame, 64))
            {
                Serial.println("[TEST] Enqueued 64 bytes of test data");
            }
            else
            {
                Serial.println("[TEST] Failed to enqueue test data");
            }
        }
        else if (cmd == 'i' || cmd == 'I')
        {
            // Print logger info
            TR_LogToFlashStats stats;
            logger.getStats(stats);
            Serial.printf("[TEST] Logging active: %s\n", stats.logging_active ? "YES" : "NO");
            Serial.printf("[TEST] Current file: %s\n", logger.currentFilename());
            Serial.printf("[TEST] Ring fill: %lu bytes\n", stats.ring_fill);
            Serial.printf("[TEST] Bytes received: %llu\n", stats.bytes_received);
            Serial.printf("[TEST] Frames received: %lu\n", stats.frames_received);
            Serial.printf("[TEST] NAND bytes written: %llu\n", stats.bytes_written_nand);
        }
        else if (cmd == 'f' || cmd == 'F')
        {
            // Format filesystem - requires confirmation
            Serial.println("[WARNING] Format filesystem? All data will be lost!");
            Serial.println("Type 'Y' to confirm or any other key to cancel:");

            // Wait for confirmation with timeout
            unsigned long start = millis();
            while (!Serial.available() && (millis() - start) < 10000) {
                delay(10);
            }

            if (Serial.available())
            {
                char confirm = Serial.read();
                if (confirm == 'Y' || confirm == 'y')
                {
                    Serial.println("[TEST] Formatting filesystem...");
                    if (logger.formatFilesystem())
                    {
                        Serial.println("[TEST] Filesystem formatted successfully!");
                    }
                    else
                    {
                        Serial.println("[TEST] Format failed!");
                    }
                }
                else
                {
                    Serial.println("[TEST] Format cancelled.");
                }
            }
            else
            {
                Serial.println("[TEST] Format timeout - cancelled.");
            }
        }
        else if (cmd == 'h' || cmd == 'H')
        {
            Serial.println("\n=== TEST COMMANDS ===");
            Serial.println("S - Start logging");
            Serial.println("E - End logging");
            Serial.println("W - Write 64 bytes of test data");
            Serial.println("I - Print logger info/stats");
            Serial.println("F - Format filesystem (WARNING: Erases all files!)");
            Serial.println("H - Show this help");
        }
    }

    // --- Active mode: FlightComputer + sensors powered on ---
    if (pwr_pin_on)
    {
        // Sim now runs on FlightComputer — I2C data always flows normally
        serviceI2CIngress();

        // Launch-triggered logging: start when NSF_LAUNCH appears in NonSensorData
        {
            static bool prev_ns_launch = false;
            const bool ns_launch = latest_non_sensor_valid &&
                                   nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
            if (ns_launch && !prev_ns_launch)
            {
                logger.startLogging();
                Serial.println("[OUT] Launch detected - logging started");
            }
            prev_ns_launch = ns_launch;
        }
        logger.service();
        serviceLoRa();

        // Check for LoRa uplink commands between TX cycles
        serviceLoRaUplink();
    }
    else
    {
        // --- Low-power mode: only BLE active, send minimal telemetry ---
        // TODO: Read battery from INA230 over I2C when hardware is ready
        // For now, battery fields will show N/A on the app

        // Idle delay to reduce CPU usage while keeping BLE stack alive
        // (esp_light_sleep breaks BLE advertising without modem-sleep coexistence config)
        delay(100);
    }

    // Auto-send config on BLE connect (rising edge)
    {
        static bool ble_was_connected = false;
        bool ble_now = ble_app.isConnected();
        if (ble_now && !ble_was_connected) {
            delay(500); // Let app subscribe to notifications
            sendCurrentConfig();
        }
        ble_was_connected = ble_now;
    }

    // Check for BLE commands
    uint8_t ble_cmd = ble_app.getCommand();
    if (ble_cmd != 0)
    {
        if (ble_cmd == 1)
        {
            // Toggle camera recording
            camera_recording_requested = !camera_recording_requested;
            setPendingCommand(camera_recording_requested ? CAMERA_START : CAMERA_STOP);
            Serial.print("[BLE] Camera toggle requested: ");
            Serial.println(camera_recording_requested ? "START" : "STOP");
        }
        else if (ble_cmd == 2)
        {
            // Send file list with pagination (5 files per page)
            // Timestamp is parsed from filename on the app side, keeping JSON compact.
            static constexpr size_t FILES_PER_PAGE = 5;
            uint8_t page = ble_app.getFileListPage();
            static TR_LogFileInfo infos[64];  // static to avoid stack overflow
            size_t total = logger.listFiles(infos, 64);
            qsort(infos, total, sizeof(TR_LogFileInfo), compareFilesDescending);  // Sort newest first

            // Calculate offset and count for this page
            size_t offset = page * FILES_PER_PAGE;
            size_t start = (offset < total) ? offset : total;
            size_t end = (start + FILES_PER_PAGE < total) ? start + FILES_PER_PAGE : total;
            size_t n = end - start;

            String json = "[";
            for (size_t i = start; i < end; ++i)
            {
                if (i > start) json += ",";
                json += "{\"name\":\"";
                json += infos[i].filename;
                json += "\",\"size\":";
                json += String(infos[i].size_bytes);
                json += "}";
            }
            json += "]";

            ble_app.sendFileList(json);

            Serial.printf("[BLE] Sent file list page %u: %u files (%u of %u, %u bytes)\n",
                          page, (unsigned)n, (unsigned)start, (unsigned)total, json.length());
        }
        else if (ble_cmd == 23)
        {
            // Toggle logging (manual start/stop from app)
            if (logger.isLoggingActive())
            {
                logger.endLogging();
                Serial.println("[BLE] Logging stopped (manual)");
            }
            else
            {
                logger.startLogging();
                Serial.println("[BLE] Logging started (manual)");
            }
        }
        else if (ble_cmd == 24)
        {
            // Servo test: set angles
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(ServoTestAnglesData))
            {
                memcpy(pending_config_data, payload, sizeof(ServoTestAnglesData));
                pending_config_data_len = sizeof(ServoTestAnglesData);
                pending_config_msg_type = SERVO_TEST_MSG;
                setPendingCommand(SERVO_TEST_PENDING);
                Serial.println("[BLE] Servo test angles received");
            }
        }
        else if (ble_cmd == 25)
        {
            // Servo test: stop
            setPendingCommand(SERVO_TEST_STOP);
            Serial.println("[BLE] Servo test stop");
        }
        else if (ble_cmd == 3)
        {
            // Delete file
            String filename = ble_app.getDeleteFilename();
            if (filename.length() > 0)
            {
                bool success = logger.deleteFile(filename.c_str());
                Serial.print("[BLE] Delete file '");
                Serial.print(filename);
                Serial.print("': ");
                Serial.println(success ? "SUCCESS" : "FAILED");

                // Send updated file list after deletion (page 0)
                static TR_LogFileInfo infos[64];  // static to avoid stack overflow
                size_t total = logger.listFiles(infos, 64);
                qsort(infos, total, sizeof(TR_LogFileInfo), compareFilesDescending);  // Sort newest first

                size_t n = (total > 5) ? 5 : total;

                String json = "[";
                for (size_t i = 0; i < n; ++i)
                {
                    if (i > 0) json += ",";
                    json += "{\"name\":\"";
                    json += infos[i].filename;
                    json += "\",\"size\":";
                    json += String(infos[i].size_bytes);
                    json += "}";
                }
                json += "]";

                ble_app.sendFileList(json);

                Serial.print("[BLE] Sent updated file list: ");
                Serial.print(n);
                Serial.print(" of ");
                Serial.print(total);
                Serial.print(" files (");
                Serial.print(json.length());
                Serial.println(" bytes)");
            }
        }

        // Handle file download requests from BLE app
        String download_filename = ble_app.getDownloadFilename();
        if (download_filename.length() > 0)
        {
            Serial.print("[BLE] Download file request: ");
            Serial.println(download_filename);

            // Dynamic chunk size based on negotiated MTU (falls back to 170 if not yet negotiated)
            const size_t chunk_data_size = ble_app.getMaxChunkDataSize();
            Serial.print("[BLE] Chunk data size: ");
            Serial.println(chunk_data_size);

            if (chunk_data_size == 0)
            {
                Serial.println("[BLE] ERROR: chunk data size is 0, aborting download");
                ble_app.sendFileChunk(0, nullptr, 0, true);  // Send EOF to unblock app
            }
            else
            {

            // Frame-aligned BLE transfer: pack complete binary frames into each
            // BLE notification so that a dropped notification only loses whole
            // frames — no frame boundaries are corrupted.
            //
            // Frame format: [AA][55][AA][55][type(1)][len(1)][payload(len)][CRC(2)]
            // Max frame = 4+1+1+255+2 = 263 bytes, always fits in one notification.
            //
            // The iOS app just appends each notification's data.  A dropped
            // notification means a few missing frames rather than a corrupted
            // region of zero-filled gaps with broken CRCs.

            const size_t MAX_FRAME_SIZE = 263;
            const size_t FLASH_READ_SIZE = 4096;
            // read_buf holds carryover from previous iteration + new flash data
            static uint8_t read_buf[FLASH_READ_SIZE + MAX_FRAME_SIZE];
            static uint8_t ble_buf[502];  // Notification payload (max MTU data)

            uint32_t file_offset = 0;     // Current position in flash file
            size_t carryover = 0;         // Bytes carried from previous flash read
            size_t ble_used = 0;          // Bytes accumulated in ble_buf
            uint32_t bytes_sent = 0;      // Total BLE bytes sent
            uint32_t frames_sent = 0;
            uint32_t start_ms = millis();
            bool eof = false;

            // Delay between every BLE notification (matches Legacy base station
            // pacing).  The bursty 3-at-a-time batch scheme overwhelmed the iOS
            // BLE notification queue, causing ~50% data loss.  A consistent
            // per-chunk delay keeps the queue shallow and reliable.
            const unsigned long CHUNK_DELAY_MS = 15;

            while (!eof)
            {
                // Read next block from flash, appended after any carryover bytes
                size_t flash_bytes_read = 0;
                if (!logger.readFileChunk(download_filename.c_str(), file_offset,
                                          read_buf + carryover, FLASH_READ_SIZE,
                                          flash_bytes_read, eof))
                {
                    Serial.println("[BLE] File read error, aborting download");
                    ble_app.sendFileChunk(bytes_sent, nullptr, 0, true);
                    break;
                }
                file_offset += flash_bytes_read;

                size_t buf_len = carryover + flash_bytes_read;
                size_t pos = 0;

                // Scan for complete frames
                while (pos + 8 <= buf_len)  // Min frame: SOF(4)+type(1)+len(1)+CRC(2)
                {
                    // Look for SOF: AA 55 AA 55
                    if (read_buf[pos]   != 0xAA || read_buf[pos+1] != 0x55 ||
                        read_buf[pos+2] != 0xAA || read_buf[pos+3] != 0x55)
                    {
                        pos++;
                        continue;
                    }

                    // Read payload length from frame header
                    uint8_t payload_len = read_buf[pos + 5];
                    size_t frame_size = 4 + 1 + 1 + payload_len + 2;

                    if (pos + frame_size > buf_len)
                    {
                        break;  // Incomplete frame — will carry over to next read
                    }

                    // Complete frame found — flush BLE buffer if this frame won't fit
                    if (ble_used > 0 && ble_used + frame_size > chunk_data_size)
                    {
                        ble_app.sendFileChunk(bytes_sent, ble_buf, ble_used, false);
                        bytes_sent += ble_used;
                        ble_used = 0;
                        delay(CHUNK_DELAY_MS);
                    }

                    // Append frame to BLE buffer
                    memcpy(ble_buf + ble_used, read_buf + pos, frame_size);
                    ble_used += frame_size;
                    frames_sent++;
                    pos += frame_size;
                }

                // Move unparsed bytes to start of buffer for next iteration
                carryover = buf_len - pos;
                if (carryover > 0 && pos > 0)
                {
                    memmove(read_buf, read_buf + pos, carryover);
                }
            }

            // Send remaining data with EOF flag
            if (ble_used > 0)
            {
                ble_app.sendFileChunk(bytes_sent, ble_buf, ble_used, true);
                bytes_sent += ble_used;
            }
            else
            {
                ble_app.sendFileChunk(bytes_sent, nullptr, 0, true);
            }

            // Redundant EOF in case the last notification was dropped
            delay(50);
            ble_app.sendFileChunk(bytes_sent, nullptr, 0, true);

            uint32_t elapsed_ms = millis() - start_ms;
            float kbps = (elapsed_ms > 0) ? (bytes_sent / 1024.0f) / (elapsed_ms / 1000.0f) : 0;
            Serial.printf("[BLE] Download complete: %lu frames, %lu bytes in %.1fs (%.1f KB/s)\n",
                          (unsigned long)frames_sent, (unsigned long)bytes_sent,
                          elapsed_ms / 1000.0f, kbps);
            } // else (chunk_data_size > 0)
        }

        // Flight simulator commands — relay to FlightComputer via I2C
        if (ble_cmd == 5)
        {
            // Configure simulation: [mass_g:4][thrust_n:4][burn_s:4][descent_rate_mps:4]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 12)
            {
                SimConfigData sim_cfg;
                float mass_g;
                memcpy(&mass_g,              payload + 0, 4);
                memcpy(&sim_cfg.thrust_n,    payload + 4, 4);
                memcpy(&sim_cfg.burn_time_s, payload + 8, 4);
                sim_cfg.mass_kg = mass_g / 1000.0f;
                sim_cfg.descent_rate_mps = 0.0f;
                if (plen >= 16) {
                    memcpy(&sim_cfg.descent_rate_mps, payload + 12, 4);
                }
                memcpy(pending_config_data, &sim_cfg, sizeof(sim_cfg));
                pending_config_data_len = sizeof(sim_cfg);
                pending_config_msg_type = SIM_CONFIG_MSG;
                setPendingCommand(SIM_CONFIG_PENDING);
                Serial.printf("[SIM] Config queued: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s\n",
                              (double)mass_g, (double)sim_cfg.thrust_n,
                              (double)sim_cfg.burn_time_s, (double)sim_cfg.descent_rate_mps);
            }
        }
        else if (ble_cmd == 6)
        {
            setPendingCommand(SIM_START_CMD);
            Serial.println("[SIM] Start queued for FlightComputer");
        }
        else if (ble_cmd == 7)
        {
            logger.endLogging();
            setPendingCommand(SIM_STOP_CMD);
            Serial.println("[SIM] Stop queued for FlightComputer (logging ended)");
        }
        else if (ble_cmd == 15)
        {
            setPendingCommand(GROUND_TEST_START);
            Serial.println("[GROUND TEST] Start queued for FlightComputer");
        }
        else if (ble_cmd == 16)
        {
            setPendingCommand(GROUND_TEST_STOP);
            Serial.println("[GROUND TEST] Stop queued for FlightComputer");
        }
        else if (ble_cmd == 8)
        {
            // Toggle power rail
            pwr_pin_on = !pwr_pin_on;

            if (pwr_pin_on)
            {
                // Disable brownout detector during power-on to ride through
                // the inrush current dip from FlightComputer + sensors.
                CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
                digitalWrite(config::PWR_PIN, HIGH);
                delay(500);  // Allow power rail to stabilize
                SET_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
                initPeripherals();  // Initialize SPI, NAND, LoRa, I2C
            }
            else
            {
                digitalWrite(config::PWR_PIN, LOW);
                peripherals_initialized = false;  // Allow reinitialization on next power-on
            }

            Serial.print("[BLE] Power rail toggled: ");
            Serial.println(pwr_pin_on ? "ON" : "OFF");
        }
        else if (ble_cmd == 9)
        {
            // Phone time sync: [year_lo][year_hi][month][day][hour][minute][second]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 7)
            {
                phone_utc_year   = payload[0] | ((uint16_t)payload[1] << 8);
                phone_utc_month  = payload[2];
                phone_utc_day    = payload[3];
                phone_utc_hour   = payload[4];
                phone_utc_minute = payload[5];
                phone_utc_second = payload[6];
                phone_sync_millis = millis();
                phone_time_valid = true;
                Serial.printf("[BLE] Time sync: %u-%02u-%02u %02u:%02u:%02u UTC\n",
                              phone_utc_year, phone_utc_month, phone_utc_day,
                              phone_utc_hour, phone_utc_minute, phone_utc_second);
            }
        }
        else if (ble_cmd == 10)
        {
            // LoRa reconfiguration: [freq:4f][bw:4f][sf:1][cr:1][txpwr:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 11)
            {
                float new_freq, new_bw;
                memcpy(&new_freq, payload + 0, 4);
                memcpy(&new_bw,   payload + 4, 4);
                uint8_t new_sf   = payload[8];
                uint8_t new_cr   = payload[9];
                int8_t  new_pwr  = (int8_t)payload[10];

                if (lora_comms.reconfigure(new_freq, new_sf, new_bw, new_cr, new_pwr))
                {
                    // Update runtime vars
                    lora_freq_mhz = new_freq;
                    lora_bw_khz   = new_bw;
                    lora_sf        = new_sf;
                    lora_cr        = new_cr;
                    lora_tx_power  = new_pwr;

                    // Persist to NVS
                    prefs.begin("lora", false);  // read-write
                    prefs.putFloat("freq",  lora_freq_mhz);
                    prefs.putFloat("bw",    lora_bw_khz);
                    prefs.putUChar("sf",    lora_sf);
                    prefs.putUChar("cr",    lora_cr);
                    prefs.putChar("txpwr",  lora_tx_power);
                    prefs.end();

                    lora_comms.startReceive();

                    Serial.printf("[BLE] LoRa reconfigured + saved: %.1f MHz SF%u BW%.0f CR%u %d dBm\n",
                                  (double)lora_freq_mhz, (unsigned)lora_sf,
                                  (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

                    // Send config readback so app can confirm the actual values applied
                    sendCurrentConfig();
                }
                else
                {
                    Serial.println("[BLE] LoRa reconfigure FAILED");
                    // Send readback with OLD config so app reverts its display
                    sendCurrentConfig();
                }
            }
        }
        else if (ble_cmd == 11)
        {
            // Rocket computer sound enable/disable: [enabled:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1)
            {
                bool enabled = (payload[0] != 0);
                setPendingCommand(enabled ? SOUNDS_ENABLE : SOUNDS_DISABLE);
                Serial.printf("[BLE] Sounds: %s (pending for RocketComputer)\n",
                              enabled ? "ENABLE" : "DISABLE");
            }
        }
        else if (ble_cmd == 12)
        {
            // Servo configuration: [bias1:2][bias2:2][bias3:2][bias4:2][hz:2][min:2][max:2] = 14 bytes
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 14)
            {
                memcpy(pending_config_data, payload, 14);
                pending_config_data_len = 14;
                pending_config_msg_type = SERVO_CONFIG_MSG;
                setPendingCommand(SERVO_CONFIG_PENDING);
                cacheServoConfig(payload, plen);
                Serial.println("[BLE] Servo config queued for RocketComputer");
            }
        }
        else if (ble_cmd == 13)
        {
            // PID configuration: [kp:4f][ki:4f][kd:4f][min_cmd:4f][max_cmd:4f] = 20 bytes
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 20)
            {
                memcpy(pending_config_data, payload, 20);
                pending_config_data_len = 20;
                pending_config_msg_type = PID_CONFIG_MSG;
                setPendingCommand(PID_CONFIG_PENDING);
                cachePIDConfig(payload, plen);
                Serial.println("[BLE] PID config queued for RocketComputer");
            }
        }
        else if (ble_cmd == 14)
        {
            // Servo control enable/disable: [enabled:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1)
            {
                bool enabled = (payload[0] != 0);
                cfg_servo_enabled = enabled;
                setPendingCommand(enabled ? SERVO_CTRL_ENABLE : SERVO_CTRL_DISABLE);
                Serial.printf("[BLE] Servo control: %s (pending for RocketComputer)\n",
                              enabled ? "ENABLE" : "DISABLE");
            }
        }
        else if (ble_cmd == 20)
        {
            // Config readback request
            sendCurrentConfig();
        }
        else if (ble_cmd == 21)
        {
            setPendingCommand(GYRO_CAL_CMD);
            Serial.println("[BLE] Gyro cal request -> FlightComputer");
        }
        else if (ble_cmd == 22)
        {
            // Gain schedule enable/disable: [enabled:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1) {
                bool enabled = (payload[0] != 0);
                cfg_gain_sched = enabled;
                setPendingCommand(enabled ? GAIN_SCHED_ENABLE : GAIN_SCHED_DISABLE);
                // Cache in OutComputer NVS for config readback
                prefs.begin("pid", false);
                prefs.putBool("gs", enabled);
                prefs.end();
                Serial.printf("[BLE] Gain schedule: %s -> FlightComputer\n",
                              enabled ? "ENABLE" : "DISABLE");
            }
        }
        else if (ble_cmd == 26)
        {
            // Roll profile set: [num_wp:1][pad:3][wp0_time:4f][wp0_angle:4f]...[wp7_time:4f][wp7_angle:4f]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(RollProfileData))
            {
                memcpy(pending_config_data, payload, sizeof(RollProfileData));
                pending_config_data_len = sizeof(RollProfileData);
                pending_config_msg_type = ROLL_PROFILE_MSG;
                setPendingCommand(ROLL_PROFILE_PENDING);
                Serial.printf("[BLE] Roll profile (%d waypoints) queued for RocketComputer\n",
                              payload[0]);
            }
            else
            {
                Serial.printf("[BLE] Roll profile payload too short (%u < %u)\n",
                              (unsigned)plen, (unsigned)sizeof(RollProfileData));
            }
        }
        else if (ble_cmd == 27)
        {
            // Roll profile clear (no payload)
            setPendingCommand(ROLL_PROFILE_CLEAR);
            Serial.println("[BLE] Roll profile CLEAR -> FlightComputer");
        }
        else if (ble_cmd == 29)
        {
            // Servo replay: send flight data sample through control loop
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(ServoReplayData))
            {
                memcpy(pending_config_data, payload, sizeof(ServoReplayData));
                pending_config_data_len = sizeof(ServoReplayData);
                pending_config_msg_type = SERVO_REPLAY_MSG;
                setPendingCommand(SERVO_REPLAY_PENDING);
                Serial.println("[BLE] Servo replay data queued");
            }
        }
        else if (ble_cmd == 30)
        {
            // Servo replay: stop
            setPendingCommand(SERVO_REPLAY_STOP);
            Serial.println("[BLE] Servo replay stop");
        }
        else if (ble_cmd == 31)
        {
            // Roll control config: [use_angle_control:1][pad:1][roll_delay_ms:2]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 4)
            {
                memcpy(pending_config_data, payload, 4);
                pending_config_data_len = 4;
                pending_config_msg_type = ROLL_CTRL_CONFIG_MSG;
                setPendingCommand(ROLL_CTRL_CONFIG_PENDING);
                cacheRollControlConfig(payload, plen);
                Serial.println("[BLE] Roll control config queued");
            }
        }
    }

    printStats();
    yield();
}

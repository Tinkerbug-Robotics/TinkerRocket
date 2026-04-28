
#include <TR_Sensor_Data_Converter.h>

SensorConverter::SensorConverter()
{
    // Default to the current SensorCollector runtime configuration.
    configureISM6HG256FullScale(ISM6LowGFullScale::FS_16G,
                                ISM6HighGFullScale::FS_256G,
                                ISM6GyroFullScale::DPS_4000);
    configureISM6HG256RotationZ(0.0f);
    configureMMC5983MARotationZ(0.0f);
}

void SensorConverter::configureISM6HG256FullScale(
                                ISM6LowGFullScale low_g_fs,
                                ISM6HighGFullScale high_g_fs,
                                ISM6GyroFullScale gyro_fs)
{
    // Sensitivity mapping assumption:
    // ISM6HG256X outputs 16-bit two's-complement samples:
    //   value = raw * (full_scale / 32768)
    // Therefore:
    //   accel: mg/LSB   = FS[g] * 1000 / 32768
    //   gyro : mdps/LSB = FS[dps] * 1000 / 32768
    //

    const float denom = 32768.0f;

    const float low_fs_g    = static_cast<float>(static_cast<uint8_t>(low_g_fs));
    const float high_fs_g   = static_cast<float>(static_cast<uint16_t>(high_g_fs));
    const float gyro_fs_dps = static_cast<float>(static_cast<uint16_t>(gyro_fs));

    const float low_mg_per_lsb = (low_fs_g  * 1000.0f) / denom;
    const float high_mg_per_lsb = (high_fs_g * 1000.0f) / denom;
    const float gyro_mdps_per_lsb = (gyro_fs_dps * 1000.0f) / denom;

    // mg/LSB -> m/s^2 per LSB
    acc_low_ms2_per_lsb  = (low_mg_per_lsb  * 1e-3f) * g_ms2;
    acc_high_ms2_per_lsb = (high_mg_per_lsb * 1e-3f) * g_ms2;

    // mdps/LSB -> dps per LSB
    gyro_dps_per_lsb = gyro_mdps_per_lsb * 1e-3f;

}

void SensorConverter::configureISM6HG256RotationZ(float rotation_z_deg)
{
    ism6_rot_z_rad = rotation_z_deg * (PI / 180.0f);
}

void SensorConverter::configureMMC5983MARotationZ(float rotation_z_deg)
{
    mmc_rot_z_rad = rotation_z_deg * (PI / 180.0f);
}

void SensorConverter::setHighGBias(float bx, float by, float bz)
{
    if (bx == hg_bias_x_ && by == hg_bias_y_ && bz == hg_bias_z_)
    {
        return;  // No change
    }
    hg_bias_x_ = bx;
    hg_bias_y_ = by;
    hg_bias_z_ = bz;
#ifdef ESP_PLATFORM
    ESP_LOGI("Converter", "High-G bias set: %.3f, %.3f, %.3f m/s²",
             (double)bx, (double)by, (double)bz);
#endif
}


// --- GNSS ---
void SensorConverter::convertGNSSData(const GNSSData& in, GNSSDataSI& out)
{
    out.time_us = in.time_us;

    out.year  = in.year;
    out.month = in.month;
    out.day   = in.day;
    out.hour  = in.hour;
    out.minute= in.minute;
    out.second= in.second;
    out.milli_second = in.milli_second;

    out.fix_mode = in.fix_mode;
    out.num_sats = in.num_sats;
    out.pdop     = (float)in.pdop_x10 / 10.0f;

    out.lat = (double)in.lat_e7 * 1e-7;
    out.lon = (double)in.lon_e7 * 1e-7;
    out.alt = (double)in.alt_mm * 1e-3;

    out.vel_e = (double)in.vel_e_mmps * 1e-3;
    out.vel_n = (double)in.vel_n_mmps * 1e-3;
    out.vel_u = (double)in.vel_u_mmps * 1e-3;

    out.horizontal_accuracy = (float)in.h_acc_m;
    out.vertical_accuracy   = (float)in.v_acc_m;
}

// --- Power ---
float SensorConverter::decodeVoltageFromInt(uint16_t raw)
{
    float ratio = (float)raw / 65535.0f;
    return ratio * 10.0f;
}

float SensorConverter::decodeCurrentFromInt(int16_t raw)
{
    float ratio = (float)raw / 32767.0f;
    return ratio * 10000.0f;
}

float SensorConverter::decodeSOCFromInt(int16_t raw)
{
    return ((float)raw * 150.0f / 32767.0f) - 25.0f;
}

uint16_t SensorConverter::encodeVoltage(float voltage_v)
{
    voltage_v = clampf(voltage_v, 0.0f, 10.0f);
    return (uint16_t)lroundf((voltage_v / 10.0f) * 65535.0f);
}

int16_t SensorConverter::encodeCurrent(float current_ma)
{
    current_ma = SensorConverter::clampf(current_ma, -10000.0f, 10000.0f);
    return (int16_t)lroundf((current_ma / 10000.0f) * 32767.0f);
}

int16_t SensorConverter::encodeSOC(float soc_pct)
{
    soc_pct = clampf(soc_pct, -25.0f, 125.0f);
    return (int16_t)lroundf((soc_pct + 25.0f) * (32767.0f / 150.0f));
}

void SensorConverter::convertPowerData(const POWERData& in, POWERDataSI& out)
{
    out.time_us = in.time_us;
    out.voltage = decodeVoltageFromInt(in.voltage_raw);
    out.current = decodeCurrentFromInt(in.current_raw);
    out.soc     = decodeSOCFromInt(in.soc_raw);
}

void SensorConverter::packPowerData(const POWERDataSI& in, POWERData& out)
{
    out.time_us     = in.time_us;
    out.voltage_raw = encodeVoltage(in.voltage);
    out.current_raw = encodeCurrent(in.current);
    out.soc_raw     = encodeSOC(in.soc);
}

// --- BMP585 ---
void SensorConverter::convertBMP585Data(const BMP585Data& in, BMP585DataSI& out)
{
    out.time_us     = in.time_us;
    out.temperature = (float)(in.temp_q16) / 65536.0f;
    out.pressure    = (float)(in.press_q6) / 64.0f;
}

// --- ISM6HG256 (low-g + high-g + gyro) ---
void SensorConverter::convertISM6HG256Data(const ISM6HG256Data& in, ISM6HG256DataSI& out)
{
    out.time_us = in.time_us;

    const double c = (double)cosf(ism6_rot_z_rad);
    const double s = (double)sinf(ism6_rot_z_rad);

    // Low-g accel (m/s^2), then rotate about +Z to board frame.
    const double lg_x = (double)in.acc_low_raw.x * (double)acc_low_ms2_per_lsb;
    const double lg_y = (double)in.acc_low_raw.y * (double)acc_low_ms2_per_lsb;
    out.low_g_acc_x = (lg_x * c) - (lg_y * s);
    out.low_g_acc_y = (lg_x * s) + (lg_y * c);
    out.low_g_acc_z = (double)in.acc_low_raw.z * (double)acc_low_ms2_per_lsb;

    // High-g accel (m/s^2), rotate about +Z to board frame, subtract bias.
    const double hg_x = (double)in.acc_high_raw.x * (double)acc_high_ms2_per_lsb;
    const double hg_y = (double)in.acc_high_raw.y * (double)acc_high_ms2_per_lsb;
    out.high_g_acc_x = (hg_x * c) - (hg_y * s) - (double)hg_bias_x_;
    out.high_g_acc_y = (hg_x * s) + (hg_y * c) - (double)hg_bias_y_;
    out.high_g_acc_z = (double)in.acc_high_raw.z * (double)acc_high_ms2_per_lsb - (double)hg_bias_z_;

    // Gyro (deg/s), then rotate about +Z to board frame.
    const double g_x = (double)in.gyro_raw.x * (double)gyro_dps_per_lsb;
    const double g_y = (double)in.gyro_raw.y * (double)gyro_dps_per_lsb;
    out.gyro_x = (g_x * c) - (g_y * s);
    out.gyro_y = (g_x * s) + (g_y * c);
    out.gyro_z = (double)in.gyro_raw.z * (double)gyro_dps_per_lsb;

}

// ---------------- MMC5983MA ----------------
static inline int32_t mmc5983ma_centered_counts(uint32_t raw18)
{
  raw18 &= 0x3FFFFu;             // keep 18 bits
  return (int32_t)raw18 - 131072; // center at 2^17
}
void SensorConverter::convertMMC5983MAData(const MMC5983MAData& in, MMC5983MADataSI& out)
{
  out.time_us = in.time_us;

  const int32_t cx = mmc5983ma_centered_counts(in.mag_x);
  const int32_t cy = mmc5983ma_centered_counts(in.mag_y);
  const int32_t cz = mmc5983ma_centered_counts(in.mag_z);

  // Gauss = centered * (8 / 131072)
  // SI: uT = Gauss * 100
  static constexpr double UT_PER_COUNT = (8.0 * 100.0) / 131072.0; // 0.006103515625

  const double mx = (double)cx * UT_PER_COUNT;
  const double my = (double)cy * UT_PER_COUNT;
  const double mz = (double)cz * UT_PER_COUNT;

  const double c = (double)cosf(mmc_rot_z_rad);
  const double s = (double)sinf(mmc_rot_z_rad);

  // Rotate sensor frame -> board frame about +Z axis.
  out.mag_x_uT = (mx * c) - (my * s);
  out.mag_y_uT = (mx * s) + (my * c);
  out.mag_z_uT = mz;
}

// --- NonSensor ---
void SensorConverter::convertNonSensorData(const NonSensorData& in, NonSensorDataSI& out)
{
    out.time_us  = in.time_us;

    // Unpack quaternion (int16 * 10000 → float, unit quaternion)
    out.q0 = (float)in.q0 / 10000.0f;
    out.q1 = (float)in.q1 / 10000.0f;
    out.q2 = (float)in.q2 / 10000.0f;
    out.q3 = (float)in.q3 / 10000.0f;

    // Derive display angles from quaternion.
    // Roll uses gimbal-lock-free body-Z azimuth (well-defined at all pitch angles).
    // Pitch and yaw use standard ZYX Euler (both well-behaved near vertical).
    const float qw = out.q0, qx = out.q1, qy = out.q2, qz = out.q3;

    // Roll — azimuth of body Z-axis in NED horizontal plane (gimbal-lock-free)
    float z_n = 2.0f * (qx * qz + qw * qy);
    float z_e = 2.0f * (qy * qz - qw * qx);
    out.roll  = -atan2f(z_e, z_n) * (180.0f / (float)M_PI);

    // Pitch — standard Euler (well-defined at vertical)
    float sinp = 2.0f * (qw * qy - qz * qx);
    out.pitch = (fabsf(sinp) >= 1.0f)
                ? copysignf(90.0f, sinp)
                : asinf(sinp) * (180.0f / (float)M_PI);

    // Yaw — standard Euler
    out.yaw   = atan2f(2.0f * (qw * qz + qx * qy),
                       1.0f - 2.0f * (qy * qy + qz * qz))
                * (180.0f / (float)M_PI);

    out.roll_cmd = (float)in.roll_cmd / 100.0f;

    // cm -> m
    out.e_pos = (double)in.e_pos * 0.01;
    out.n_pos = (double)in.n_pos * 0.01;
    out.u_pos = (double)in.u_pos * 0.01;

    // cm/s -> m/s
    out.e_vel = (double)in.e_vel * 0.01;
    out.n_vel = (double)in.n_vel * 0.01;
    out.u_vel = (double)in.u_vel * 0.01;

    // Altitude rate from FlightComputer KF; other metrics derived on OutComputer.
    out.pressure_alt  = 0.0f;
    out.altitude_rate = (float)in.baro_alt_rate_dmps * 0.1f;  // dm/s -> m/s
    out.max_alt       = 0.0f;
    out.max_speed     = 0.0f;

    out.alt_landed_flag   = (in.flags & NSF_ALT_LANDED) != 0;
    out.alt_apogee_flag   = (in.flags & NSF_ALT_APOGEE) != 0;
    out.vel_u_apogee_flag = (in.flags & NSF_VEL_APOGEE) != 0;
    out.launch_flag       = (in.flags & NSF_LAUNCH) != 0;

    out.rocket_state = (RocketState)in.rocket_state;
}

// --- LoRa pack/unpack ---
void SensorConverter::packLoRa(const LoRaDataSI& in, LoRaData& out)
{
    // Routing header
    out.network_id       = in.network_id;
    out.rocket_id        = in.rocket_id;
    out.next_channel_idx = in.next_channel_idx;

    // num_sats (bits 0-6) + logging_active (bit 7)
    out.num_sats = (uint8_t)lroundi32(clampf((float)in.num_sats, 0.f, 127.f));
    if (in.logging_active) out.num_sats |= LORA_LOGGING_BIT;

    // pdop -> u8 0..100
    out.pdop_u8 = (uint8_t)lroundi32(clampf(in.pdop, 0.f, 100.f));

    // ECEF position (int24 meters)
    double ex = clampd(in.ecef_x, -7000000.0, 7000000.0);
    double ey = clampd(in.ecef_y, -7000000.0, 7000000.0);
    double ez = clampd(in.ecef_z, -7000000.0, 7000000.0);
    out.ecef_x_m = i24_from_i32(lroundi32(ex));
    out.ecef_y_m = i24_from_i32(lroundi32(ey));
    out.ecef_z_m = i24_from_i32(lroundi32(ez));

    // horizontal_accuracy -> u8 0..100
    out.hacc_u8 = (uint8_t)lroundi32(clampf(in.horizontal_accuracy, 0.f, 100.f));

    // flags + rocket_state in 1 byte
    uint8_t packed = 0;
    if (in.launch_flag)       packed |= LORA_LAUNCH;
    if (in.vel_u_apogee_flag) packed |= LORA_VEL_APOGEE;
    if (in.alt_apogee_flag)   packed |= LORA_ALT_APOGEE;
    if (in.alt_landed_flag)   packed |= LORA_ALT_LANDED;
    if (in.camera_recording)  packed |= LORA_CAMERA_REC;
    uint8_t state = (uint8_t)(in.rocket_state > 4 ? 4 : in.rocket_state);
    packed |= (state & 0x07u) << LORA_STATE_SHIFT; // b4..b6
    out.flags_state = packed;

    // acceleration (×10), −400..400 → i16
    auto enc_acc = [](float a)->int16_t {
        a = SensorConverter::clampf(a, -400.f, 400.f);
        return (int16_t)lroundf(a * 10.f);
    };
    out.acc_x_x10 = enc_acc(in.acc_x);
    out.acc_y_x10 = enc_acc(in.acc_y);
    out.acc_z_x10 = enc_acc(in.acc_z);

    // gyro (×10), −4500..4500 → i16
    auto enc_gyro = [](float g)->int16_t {
        g = SensorConverter::clampf(g, -4500.f, 4500.f);
        return (int16_t)lroundf(g * 10.f);
    };
    out.gyro_x_x10 = enc_gyro(in.gyro_x);
    out.gyro_y_x10 = enc_gyro(in.gyro_y);
    out.gyro_z_x10 = enc_gyro(in.gyro_z);

    // temperature ×10 → i16
    {
        float t = clampf(in.temp, -40.f, 200.f);
        out.temp_x10 = (int16_t)lroundf(t * 10.f);
    }

    // voltage -> u8 2..10V
    out.voltage_u8 = encodeVoltage_2_10_01(in.voltage);

    // current mA → i16
    {
        float c = clampf(in.current, -10000.f, 10000.f);
        out.current_ma = (int16_t)lroundf(c);
    }

    // soc % → i8
    {
        float s = clampf(in.soc, -25.f, 125.f);
        int val = (int)lroundf(s);
        if (val < -128) val = -128;
        if (val > 127)  val = 127;
        out.soc_i8 = (int8_t)val;
    }

    // pressure_alt → i24 (meters)
    {
        float pa = SensorConverter::clampf(in.pressure_alt, -1000.f, 100000.f);
        out.pressure_alt_m = i24_from_i32(lroundi32(pa));
    }

    // altitude_rate → i16
    {
        float ar = clampf(in.altitude_rate, -2000.f, 2000.f);
        out.altitude_rate = (int16_t)lroundf(ar);
    }

    // max_alt → i24
    {
        float ma = SensorConverter::clampf(in.max_alt, -1000.f, 400000.f);
        out.max_alt_m = i24_from_i32(lroundi32(ma));
    }

    // max_speed → i16
    {
        float ms = clampf(in.max_speed, 0.f, 4000.f);
        out.max_speed = (int16_t)lroundf(ms);
    }

    // roll/pitch/yaw -> i16 centidegrees
    auto enc_angle_cd = [](float a, float lim)->int16_t {
        if (a >  lim) a =  lim;
        if (a < -lim) a = -lim;
        return (int16_t)lroundf(a * 100.f);
    };
    out.roll_cd  = enc_angle_cd(in.roll,  180.f);
    out.pitch_cd = enc_angle_cd(in.pitch,  90.f);
    out.yaw_cd   = enc_angle_cd(in.yaw,   180.f);

    // quaternion × 10000 → i16
    auto enc_quat = [](float q)->int16_t {
        q = SensorConverter::clampf(q, -1.f, 1.f);
        return (int16_t)lroundf(q * 10000.f);
    };
    out.q0 = enc_quat(in.q0);
    out.q1 = enc_quat(in.q1);
    out.q2 = enc_quat(in.q2);
    out.q3 = enc_quat(in.q3);

    // speed → i16
    {
        float sp = clampf(in.speed, 0.f, 4000.f);
        out.speed = (int16_t)lroundf(sp);
    }
}

void SensorConverter::unpackLoRa(const LoRaData& in, LoRaDataSI& out)
{
    // Routing header
    out.network_id       = in.network_id;
    out.rocket_id        = in.rocket_id;
    out.next_channel_idx = in.next_channel_idx;

    out.num_sats = in.num_sats & 0x7F;  // bits 0-6
    out.logging_active = (in.num_sats & LORA_LOGGING_BIT) != 0;
    out.pdop = (float)in.pdop_u8;

    out.ecef_x = (double)i24_to_i32(in.ecef_x_m);
    out.ecef_y = (double)i24_to_i32(in.ecef_y_m);
    out.ecef_z = (double)i24_to_i32(in.ecef_z_m);

    out.horizontal_accuracy = (float)in.hacc_u8;

    out.launch_flag       = (in.flags_state & LORA_LAUNCH) != 0;
    out.vel_u_apogee_flag = (in.flags_state & LORA_VEL_APOGEE) != 0;
    out.alt_apogee_flag   = (in.flags_state & LORA_ALT_APOGEE) != 0;
    out.alt_landed_flag   = (in.flags_state & LORA_ALT_LANDED) != 0;
    out.camera_recording  = (in.flags_state & LORA_CAMERA_REC) != 0;
    out.rocket_state      = (in.flags_state >> LORA_STATE_SHIFT) & 0x07u;

    out.acc_x = (float)in.acc_x_x10 / 10.0f;
    out.acc_y = (float)in.acc_y_x10 / 10.0f;
    out.acc_z = (float)in.acc_z_x10 / 10.0f;

    out.gyro_x = (float)in.gyro_x_x10 / 10.0f;
    out.gyro_y = (float)in.gyro_y_x10 / 10.0f;
    out.gyro_z = (float)in.gyro_z_x10 / 10.0f;

    out.temp   = (float)in.temp_x10 / 10.0f;

    out.voltage = decodeVoltage_2_10_01(in.voltage_u8);
    out.current = (float)in.current_ma;
    out.soc     = (float)in.soc_i8;

    out.pressure_alt  = (float)i24_to_i32(in.pressure_alt_m);
    out.altitude_rate = (float)in.altitude_rate;
    out.max_alt       = (float)i24_to_i32(in.max_alt_m);
    out.max_speed     = (float)in.max_speed;

    out.roll  = (float)in.roll_cd  / 100.0f;
    out.pitch = (float)in.pitch_cd / 100.0f;
    out.yaw   = (float)in.yaw_cd   / 100.0f;

    out.q0 = (float)in.q0 / 10000.0f;
    out.q1 = (float)in.q1 / 10000.0f;
    out.q2 = (float)in.q2 / 10000.0f;
    out.q3 = (float)in.q3 / 10000.0f;

    out.speed = (float)in.speed;

    // Not transmitted in LoRaData (superset fields)
    out.base_station_voltage = 0.0f;
    out.base_station_current = 0.0f;
    out.base_station_soc     = 0.0f;
    out.rssi = 0.0f;
    out.snr  = 0.0f;
}

void SensorConverter::packLoRaData(const LoRaDataSI& in, uint8_t out_bytes[SIZE_OF_LORA_DATA])
{
    LoRaData p{};
    packLoRa(in, p);
    memcpy(out_bytes, &p, sizeof(LoRaData));
}

void SensorConverter::unpackLoRa(const uint8_t in_bytes[SIZE_OF_LORA_DATA], LoRaDataSI& out)
{
    LoRaData p{};
    memcpy(&p, in_bytes, sizeof(LoRaData));
    unpackLoRa(p, out);
}

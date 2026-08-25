// Golden-vector fixture generation — see wire_fixtures_lib.h for the contract.
//
// Layout notes live next to each family.  Values are chosen to be hostile to
// naive ports: u32 fields above INT32_MAX (sign traps), negative values in
// every signed slot (endianness + sign-extension), 18-bit mag counts above
// 0x20000 (masking), i8 TX power below zero (bitPattern round-trips).

#include "wire_fixtures_lib.h"

#include "RocketComputerTypes.h"
#include "CRC.h"
#include "TR_Sensor_Data_Converter.h"   // #850: real LoRa packers for the BS log golden

#include <cstdio>
#include <cstring>

static_assert(sizeof(GNSSData) == 42, "layout drift: regen fixtures + bump manifest");
// MagCalStatusData's full image appears ONLY in the 0xCA ladder (its top rung
// is sizeof-derived below, so append-only growth changes generated output and
// trips the freshness diff) — this pin is the second line of defense: a new
// fw era must add its rung deliberately, not silently.
static_assert(sizeof(MagCalStatusData) == 36,
              "new MagCal era: extend the 0xCA ladder + regen fixtures");

namespace tr_wire_fixtures {
namespace {

// Little-endian host required: struct byte images are the wire format.
static const union { uint16_t u; uint8_t b[2]; } kEnd{0x0102};
bool hostIsLittleEndian() { return kEnd.b[0] == 0x02; }

// ---------------------------------------------------------------- helpers ---

template <class T>
std::vector<uint8_t> bytesOf(const T& v) {
    std::vector<uint8_t> out(sizeof(T));
    std::memcpy(out.data(), &v, sizeof(T));
    return out;
}

std::vector<uint8_t> prefix(const std::vector<uint8_t>& v, size_t n) {
    return std::vector<uint8_t>(v.begin(), v.begin() + n);
}

void append(std::vector<uint8_t>& out, const std::vector<uint8_t>& v) {
    out.insert(out.end(), v.begin(), v.end());
}

void appendU8(std::vector<uint8_t>& out, uint8_t v) { out.push_back(v); }
void appendI8(std::vector<uint8_t>& out, int8_t v) { out.push_back(static_cast<uint8_t>(v)); }
void appendU16(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(v & 0xFF);
    out.push_back((v >> 8) & 0xFF);
}
void appendF32(std::vector<uint8_t>& out, float v) {
    uint8_t b[4];
    std::memcpy(b, &v, 4);
    out.insert(out.end(), b, b + 4);
}
void appendF64(std::vector<uint8_t>& out, double v) {
    uint8_t b[8];
    std::memcpy(b, &v, 8);
    out.insert(out.end(), b, b + 8);
}

std::vector<uint8_t> strBytes(const std::string& s) {
    return std::vector<uint8_t>(s.begin(), s.end());
}

// Minimal ordered JSON writer.  Key order is declaration order — stable output
// is part of the fixture contract (the freshness test byte-compares sidecars).
class Json {
public:
    Json() : s_("{") {}
    Json& u(const char* k, uint64_t v) { key(k); s_ += std::to_string(v); return *this; }
    Json& i(const char* k, int64_t v) { key(k); s_ += std::to_string(v); return *this; }
    Json& b(const char* k, bool v) { key(k); s_ += v ? "true" : "false"; return *this; }
    // %.9g / %.17g round-trip float/double exactly through decimal text.
    Json& f(const char* k, float v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
        key(k); s_ += buf; return *this;
    }
    Json& d(const char* k, double v) {
        char buf[40];
        std::snprintf(buf, sizeof(buf), "%.17g", v);
        key(k); s_ += buf; return *this;
    }
    Json& str(const char* k, const std::string& v) {
        key(k); s_ += '"'; s_ += v; s_ += '"'; return *this;
    }
    // Pre-encoded value (array / nested object).
    Json& raw(const char* k, const std::string& v) { key(k); s_ += v; return *this; }
    std::string done() const { return s_ + "}\n"; }

private:
    void key(const char* k) {
        if (s_.size() > 1) s_ += ",";
        s_ += "\n \""; s_ += k; s_ += "\": ";
    }
    std::string s_;
};

template <class T, class Fn>
std::string jsonArray(const std::vector<T>& v, Fn fmt) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += ",";
        s += fmt(v[i]);
    }
    return s + "]";
}

std::string intArray(const std::vector<int64_t>& v) {
    return jsonArray(v, [](int64_t x) { return std::to_string(x); });
}

// [AA 55 AA 55][type][len u8][payload][CRC16 hi][CRC16 lo] — mirrors
// TR_I2C_Interface::packMessage exactly, with the CRC from the real
// components/CRC library (poly 0x8001, init 0, unreflected).
std::vector<uint8_t> frame(uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> f{0xAA, 0x55, 0xAA, 0x55};
    f.push_back(type);
    f.push_back(static_cast<uint8_t>(payload.size()));
    append(f, payload);
    const uint16_t crc = calcCRC16(f.data() + 4, static_cast<int>(2 + payload.size()));
    f.push_back((crc >> 8) & 0xFF);
    f.push_back(crc & 0xFF);
    return f;
}

// ------------------------------------------------------- canonical structs ---
// One canonical instance per struct.  Change a value here → regen → all three
// test suites re-pin.  Values deliberately exercise sign/endian/mask traps.

GNSSData canonicalGnss() {
    GNSSData g{};
    g.time_us = 3600123456u;  // > INT32_MAX: catches signed-int u32 reads
    g.year = 2026; g.month = 7; g.day = 23;
    g.hour = 14; g.minute = 5; g.second = 9; g.milli_second = 987;
    g.fix_mode = 3; g.num_sats = 17; g.pdop_x10 = 13;
    g.lat_e7 = 377749000; g.lon_e7 = -1224194000; g.alt_mm = 123456;
    g.vel_e_mmps = -1500; g.vel_n_mmps = 2500; g.vel_u_mmps = -50;
    g.h_acc_m = 4; g.v_acc_m = 7;
    return g;
}

ISM6HG256Data canonicalImu() {
    ISM6HG256Data d{};
    d.time_us = 2147483648u;  // exactly 2^31
    d.acc_low_raw = {-1000, 2000, -32768};
    d.acc_high_raw = {100, -200, 32767};
    d.gyro_raw = {-15000, 1000, 29999};
    return d;
}

BMP585Data canonicalBaro() {
    BMP585Data d{};
    d.time_us = 4000000000u;
    d.temp_q16 = 1540096;   // 23.5 °C * 65536
    d.press_q6 = 6484800u;  // 101325 Pa * 64
    return d;
}

MMC5983MAData canonicalMmc() {
    MMC5983MAData d{};
    d.time_us = 1111111u;
    d.mag_x = 0x0002ABCDu;  // > 0x20000: catches missing 18-bit mask
    d.mag_y = 100000u;
    d.mag_z = 131072u;      // exactly center
    return d;
}

IIS2MDCData canonicalIis() {
    IIS2MDCData d{};
    d.time_us = 999999u;
    d.mag_x = -2000; d.mag_y = 0; d.mag_z = 1234;
    return d;
}

POWERData canonicalPower() {
    POWERData d{};
    d.time_us = 123456789u;
    d.voltage_raw = 27000; d.current_raw = -1234; d.soc_raw = 15000;
    // #850 v2 tail. Deliberately distinct, non-zero, and not byte-symmetric so
    // a v1 decoder reading past its length, or a byte-swap, both show up.
    d.cam_ma = 1480; d.servo_ma = 2960;
    return d;
}

NonSensorData canonicalNonSensor() {
    NonSensorData d{};
    d.time_us = 5000000u;
    d.q0 = 9239; d.q1 = -2588; d.q2 = 1913; d.q3 = 2706;
    d.roll_cmd = -4500;
    d.e_pos = 12345; d.n_pos = -6789; d.u_pos = 101112;
    d.e_vel = -100; d.n_vel = 250; d.u_vel = -9999;
    d.flags = 0x2D;         // landed | vel_u_apogee | launch | guidance_active
    d.rocket_state = 6;
    d.baro_alt_rate_dmps = -123;
    d.pyro_status = 0x0A;   // PSF: ch1 fired + ch2 fired (bits 1, 3)
    d.apogee_flags = 0x05;  // gps_apogee | master apogee
    d.sensor_health = 0x00F0A5C3u;  // gnssAbsent bits 22-23 = 3 (BAD/active)
    d.ekf_ticks = 54321;
    return d;
}

OutStatusQueryData canonicalStatusQuery() {
    OutStatusQueryData d{};
    d.ism6_low_g_fs_g = 16; d.ism6_high_g_fs_g = 256; d.ism6_gyro_fs_dps = 4000;
    d.ism6_rot_z_cdeg = 9000; d.mmc_rot_z_cdeg = -4500;
    d.format_version = 6;
    d.hg_bias_x_cmss = -50; d.hg_bias_y_cmss = 75; d.hg_bias_z_cmss = -100;
    d.b2r_code = 4; d.b2r_mode = 2;
    d.b2r_q[0] = 9239; d.b2r_q[1] = 0; d.b2r_q[2] = -3827; d.b2r_q[3] = 0;
    d.iis2mdc_rot_z_cdeg = 18000;
    d.tgt_lat_deg = 37.7749f; d.tgt_lon_deg = -122.4194f;
    d.tgt_alt_m = 250; d.tgt_seq = 3;
    // Adjacent u8s get DISTINCT values so a field-swap/off-by-one decode is
    // visible (semantically valid: status describes the still-active previous
    // target, last_rc the most recent — rejected — cmd 28).
    d.tgt_status = GUID_TGT_GEO_ACTIVE; d.tgt_last_rc = GUID_RC_REJ_RADIUS;
    // v6: nonzero so the mag-type byte can't pass by being mistaken for the
    // zeroed pad a truncation would leave.
    d.mag_type = MAG_TYPE_QMC5883P;
    return d;
}

PyroConfigData canonicalPyro() {
    PyroConfigData p{};
    p.ch1_enabled = 1; p.ch1_trigger_mode = PYRO_TRIGGER_TIME_AFTER_APOGEE;  p.ch1_trigger_value = 2.5f;
    p.ch2_enabled = 1; p.ch2_trigger_mode = PYRO_TRIGGER_ALTITUDE_ON_DESCENT; p.ch2_trigger_value = 150.0f;
    p.ch3_enabled = 0; p.ch3_trigger_mode = PYRO_TRIGGER_TIME_AFTER_APOGEE;  p.ch3_trigger_value = 0.0f;
    p.ch4_enabled = 1; p.ch4_trigger_mode = PYRO_TRIGGER_TIME_AFTER_APOGEE;  p.ch4_trigger_value = 8.75f;
    return p;
}

RollProfileData canonicalRollProfile() {
    RollProfileData r{};
    r.num_waypoints = 3;
    r.waypoints[0] = {0.5f, 0.0f, ROLL_SEG_ANGLE};
    r.waypoints[1] = {2.0f, 180.0f, ROLL_SEG_ANGLE};
    r.waypoints[2] = {4.0f, -90.0f, ROLL_SEG_ANGLE};
    return r;
}

FlightSettingsData canonicalFlightSettings() {
    FlightSettingsData s{};
    s.time_us = 250000123u;
    s.version = FlightSettingsData::VERSION;
    s.flags = 0x4F;  // angle_ctl | gain_sched | guidance | servo_en | station_keep
    s.roll_delay_ms = 1500;
    s.kp = 0.8f; s.ki = 0.15f; s.kd = 0.05f; s.d_lpf_hz = 25.0f;
    s.min_cmd_deg = -15.0f; s.max_cmd_deg = 15.0f;
    s.kp_angle = 2.5f; s.kp_angle_rate_cap_dps = 180.0f;
    s.gs_v_ref = 50.0f; s.gs_v_min = 15.0f; s.gs_scale_cap = 3.0f;
    s.roll_rate_set_point = 0.0f;
    s.ism6_low_g_fs_g = 16; s.ism6_high_g_fs_g = 256; s.ism6_gyro_fs_dps = 4000;
    s.servo_bias_us[0] = -120; s.servo_bias_us[1] = 35;
    s.servo_bias_us[2] = 0;    s.servo_bias_us[3] = 88;
    s.servo_hz = 333; s.servo_min_us = 1000; s.servo_max_us = 2000;
    s.camera_type = 2;
    s.pyro = canonicalPyro();
    std::strncpy(s.fw_git_sha, "abc123def", sizeof(s.fw_git_sha));
    s.roll_profile = canonicalRollProfile();
    s.b2r_code = 4; s.b2r_mode = 2; s.b2r_residual_cdeg = 150;
    s.b2r_q[0] = 9239; s.b2r_q[1] = 0; s.b2r_q[2] = -3827; s.b2r_q[3] = 0;
    s.fin_min_deg = -12.5f; s.fin_max_deg = 12.5f;
    s.ism6_update_rate_hz = 1920;
    s.guid_tgt_e_m = 25.5f; s.guid_tgt_n_m = -30.25f;
    s.guid_tgt_src = GUID_TGT_GEO_ACTIVE;
    return s;
}

MagCalStatusData canonicalMagCal() {
    MagCalStatusData m{};
    m.time_us = 777777u;
    m.sub_type = 3; m.coverage_bins = 18; m.sample_count = 642;
    m.inst_field_uT_x10 = 5234;
    m.offset_x = -321; m.offset_y = 456; m.offset_z = -789;
    m.field_R_uT_x10 = 482; m.residual_uT_x10 = 37;
    m.reject_code = 0; m._pad = 0;
    m.coverage_mask = 0x0003FFFFu;
    m.inst_x_lsb = -100; m.inst_y_lsb = 200; m.inst_z_lsb = -300;
    m.partial_mask = 0x000C0000u;  // disjoint from coverage_mask by contract
    return m;
}

SensorCalStatusData canonicalSensorCal() {
    SensorCalStatusData s{};
    s.valid = 1;
    s.gyro_x = -12; s.gyro_y = 34; s.gyro_z = -56;
    s.hg_x = 0.25f; s.hg_y = -0.5f; s.hg_z = 9.81f;
    return s;
}

RocketStorageStatsData canonicalRocketStorage() {
    RocketStorageStatsData r{};
    r.flight_region_blocks = 2012; r.used_blocks = 310; r.free_blocks = 1650;
    r.bad_blocks = 12; r.system_blocks = 40; r.flight_count = 27;
    r.block_size_kb = 256;
    r.flags = RSS_FLAG_INITIALIZED | RSS_FLAG_AUTO_EVICTED;
    return r;
}

BaseStationStorageStatsData canonicalBsStorage() {
    BaseStationStorageStatsData b{};
    b.total_bytes = 512000000000ull;
    b.used_bytes = 100200300400ull;
    b.free_bytes = 411799699600ull;
    b.backend = 2; b.flags = 1;  // distinct adjacent u8s (swap-visible)
    return b;
}

// ------------------------------------------------------------- the corpus ---

struct Builder {
    FixtureMap out;
    std::string manifest;  // accumulated manifest entries

    void add(const std::string& family, const std::string& name,
             const std::vector<uint8_t>& bytes, const std::string& sidecarJson,
             const std::string& notes) {
        const std::string base = family + "/" + name;
        out[base] = bytes;
        if (!sidecarJson.empty()) {
            std::string side = base;
            const auto dot = side.rfind('.');
            side = side.substr(0, dot) + ".expected.json";
            out[side] = strBytes(sidecarJson);
        }
        if (!manifest.empty()) manifest += ",";
        Json e;
        e.str("file", base).u("size", bytes.size()).str("notes", notes);
        std::string entry = e.done();
        entry.pop_back();  // trailing newline stays out of the array
        manifest += "\n " + entry;
    }
};

std::string gnssSidecar(const GNSSData& g) {
    return Json()
        .u("time_us", g.time_us)
        .u("year", g.year).u("month", g.month).u("day", g.day)
        .u("hour", g.hour).u("minute", g.minute).u("second", g.second)
        .u("milli_second", g.milli_second)
        .u("fix_mode", g.fix_mode).u("num_sats", g.num_sats).u("pdop_x10", g.pdop_x10)
        .i("lat_e7", g.lat_e7).i("lon_e7", g.lon_e7).i("alt_mm", g.alt_mm)
        .i("vel_e_mmps", g.vel_e_mmps).i("vel_n_mmps", g.vel_n_mmps).i("vel_u_mmps", g.vel_u_mmps)
        .u("h_acc_m", g.h_acc_m).u("v_acc_m", g.v_acc_m)
        .done();
}

std::string nonSensorSidecar(const NonSensorData& d, size_t presentBytes) {
    Json j;
    j.u("present_bytes", presentBytes)
        .u("time_us", d.time_us)
        .i("q0", d.q0).i("q1", d.q1).i("q2", d.q2).i("q3", d.q3)
        .i("roll_cmd", d.roll_cmd)
        .i("e_pos", d.e_pos).i("n_pos", d.n_pos).i("u_pos", d.u_pos)
        .i("e_vel", d.e_vel).i("n_vel", d.n_vel).i("u_vel", d.u_vel)
        .u("flags", d.flags).u("rocket_state", d.rocket_state)
        .i("baro_alt_rate_dmps", d.baro_alt_rate_dmps)
        .u("pyro_status", d.pyro_status);
    // Ladder semantics: fields beyond present_bytes are ABSENT (Kotlin/Swift
    // decode nil/default), not zero.  Sidecar only lists present fields.
    if (presentBytes >= 44) j.u("apogee_flags", d.apogee_flags);
    if (presentBytes >= 48) j.u("sensor_health", d.sensor_health);
    if (presentBytes >= 50) j.u("ekf_ticks", d.ekf_ticks);
    return j.done();
}

// Every ladder rung gets a sidecar (the corpus contract: NO .bin without its
// .expected.json).  `version` is the PATCHED byte in the truncated image, not
// the canonical struct's — the decoders gate on both length and version.
std::string statusQuerySidecar(const OutStatusQueryData& d, size_t presentBytes,
                               uint8_t version) {
    Json j;
    j.u("present_bytes", presentBytes)
        .u("ism6_low_g_fs_g", d.ism6_low_g_fs_g)
        .u("ism6_high_g_fs_g", d.ism6_high_g_fs_g)
        .u("ism6_gyro_fs_dps", d.ism6_gyro_fs_dps)
        .i("ism6_rot_z_cdeg", d.ism6_rot_z_cdeg)
        .i("mmc_rot_z_cdeg", d.mmc_rot_z_cdeg)
        .u("format_version", version)
        .i("hg_bias_x_cmss", d.hg_bias_x_cmss)
        .i("hg_bias_y_cmss", d.hg_bias_y_cmss)
        .i("hg_bias_z_cmss", d.hg_bias_z_cmss)
        .u("b2r_code", d.b2r_code).u("b2r_mode", d.b2r_mode)
        .raw("b2r_q", intArray({d.b2r_q[0], d.b2r_q[1], d.b2r_q[2], d.b2r_q[3]}));
    if (presentBytes >= 28) j.i("iis2mdc_rot_z_cdeg", d.iis2mdc_rot_z_cdeg);
    if (presentBytes >= 41) {
        j.f("tgt_lat_deg", d.tgt_lat_deg).f("tgt_lon_deg", d.tgt_lon_deg)
            .i("tgt_alt_m", d.tgt_alt_m).u("tgt_seq", d.tgt_seq)
            .u("tgt_status", d.tgt_status).u("tgt_last_rc", d.tgt_last_rc);
    }
    if (presentBytes >= 42) j.u("mag_type", d.mag_type);
    return j.done();
}

std::string flightSettingsSidecar(const FlightSettingsData& s, size_t presentBytes,
                                  uint8_t version) {
    Json j;
    j.u("present_bytes", presentBytes)
        .u("time_us", s.time_us).u("version", version).u("flags", s.flags)
        .u("roll_delay_ms", s.roll_delay_ms)
        .f("kp", s.kp).f("ki", s.ki).f("kd", s.kd).f("d_lpf_hz", s.d_lpf_hz)
        .f("min_cmd_deg", s.min_cmd_deg).f("max_cmd_deg", s.max_cmd_deg)
        .f("kp_angle", s.kp_angle).f("kp_angle_rate_cap_dps", s.kp_angle_rate_cap_dps)
        .f("gs_v_ref", s.gs_v_ref).f("gs_v_min", s.gs_v_min).f("gs_scale_cap", s.gs_scale_cap)
        .f("roll_rate_set_point", s.roll_rate_set_point)
        .u("ism6_low_g_fs_g", s.ism6_low_g_fs_g)
        .u("ism6_high_g_fs_g", s.ism6_high_g_fs_g)
        .u("ism6_gyro_fs_dps", s.ism6_gyro_fs_dps)
        .raw("servo_bias_us", intArray({s.servo_bias_us[0], s.servo_bias_us[1],
                                        s.servo_bias_us[2], s.servo_bias_us[3]}))
        .i("servo_hz", s.servo_hz).i("servo_min_us", s.servo_min_us)
        .i("servo_max_us", s.servo_max_us)
        .u("camera_type", s.camera_type)
        .str("fw_git_sha", s.fw_git_sha)
        .u("roll_profile_num_waypoints", s.roll_profile.num_waypoints);
    if (presentBytes >= 200) {
        j.u("b2r_code", s.b2r_code).u("b2r_mode", s.b2r_mode)
            .i("b2r_residual_cdeg", s.b2r_residual_cdeg);
    }
    if (presentBytes >= 208) j.f("fin_min_deg", s.fin_min_deg).f("fin_max_deg", s.fin_max_deg);
    if (presentBytes >= 210) j.u("ism6_update_rate_hz", s.ism6_update_rate_hz);
    if (presentBytes >= 219) {
        j.f("guid_tgt_e_m", s.guid_tgt_e_m).f("guid_tgt_n_m", s.guid_tgt_n_m)
            .u("guid_tgt_src", s.guid_tgt_src);
    }
    return j.done();
}

std::string magCalSidecar(const MagCalStatusData& m, size_t presentBytes) {
    Json j;
    j.u("present_bytes", presentBytes)
        .u("time_us", m.time_us)
        .u("sub_type", m.sub_type).u("coverage_bins", m.coverage_bins)
        .u("sample_count", m.sample_count).u("inst_field_uT_x10", m.inst_field_uT_x10)
        .i("offset_x", m.offset_x).i("offset_y", m.offset_y).i("offset_z", m.offset_z)
        .u("field_R_uT_x10", m.field_R_uT_x10).u("residual_uT_x10", m.residual_uT_x10)
        .u("reject_code", m.reject_code);
    if (presentBytes >= 26) j.u("coverage_mask", m.coverage_mask);
    if (presentBytes >= 32) {
        j.i("inst_x_lsb", m.inst_x_lsb).i("inst_y_lsb", m.inst_y_lsb).i("inst_z_lsb", m.inst_z_lsb);
    }
    if (presentBytes >= 36) j.u("partial_mask", m.partial_mask);
    return j.done();
}

void buildLogframes(Builder& b) {
    const auto gnss = canonicalGnss();
    b.add("logframes", "gnss_42.bin", bytesOf(gnss), gnssSidecar(gnss),
          "GNSSData, msg 0xA1; u32 time > INT32_MAX");

    const auto imu = canonicalImu();
    b.add("logframes", "imu_ism6_22.bin", bytesOf(imu),
          Json().u("time_us", imu.time_us)
              .raw("acc_low_raw", intArray({imu.acc_low_raw.x, imu.acc_low_raw.y, imu.acc_low_raw.z}))
              .raw("acc_high_raw", intArray({imu.acc_high_raw.x, imu.acc_high_raw.y, imu.acc_high_raw.z}))
              .raw("gyro_raw", intArray({imu.gyro_raw.x, imu.gyro_raw.y, imu.gyro_raw.z}))
              .done(),
          "ISM6HG256Data, msg 0xA2 (22 B = Mini; 36 B payload = legacy ICM45686)");

    const auto baro = canonicalBaro();
    b.add("logframes", "baro_bmp585_12.bin", bytesOf(baro),
          Json().u("time_us", baro.time_us).i("temp_q16", baro.temp_q16)
              .u("press_q6", baro.press_q6).done(),
          "BMP585Data, msg 0xA3 (12 B = BMP585; 10 B = legacy MS5611)");

    const auto mmc = canonicalMmc();
    b.add("logframes", "mag_mmc5983_16.bin", bytesOf(mmc),
          Json().u("time_us", mmc.time_us)
              .u("mag_x", mmc.mag_x).u("mag_y", mmc.mag_y).u("mag_z", mmc.mag_z)
              .done(),
          "MMC5983MAData, msg 0xA4; mag_x > 0x20000 catches missing 18-bit mask");

    const auto iis = canonicalIis();
    b.add("logframes", "mag_iis2mdc_10.bin", bytesOf(iis),
          Json().u("time_us", iis.time_us)
              .i("mag_x", iis.mag_x).i("mag_y", iis.mag_y).i("mag_z", iis.mag_z)
              .done(),
          "IIS2MDCData, msg 0xD1");

    // POWER length ladder (#850): 10 (v1) / 14 (v2, +cam_ma +servo_ma).  v1 is
    // a faithful prefix of the append-only struct, and logs written before #850
    // are 10 B forever — so both lengths stay pinned here.
    const auto pwr = canonicalPower();
    const auto pwrFull = bytesOf(pwr);
    b.add("logframes", "power_14.bin", pwrFull,
          Json().u("time_us", pwr.time_us).u("voltage_raw", pwr.voltage_raw)
              .i("current_raw", pwr.current_raw).i("soc_raw", pwr.soc_raw)
              .u("cam_ma", pwr.cam_ma).u("servo_ma", pwr.servo_ma)
              .done(),
          "POWERData v2 (+cam_ma +servo_ma), msg 0xA6");
    b.add("logframes", "power_10.bin", prefix(pwrFull, 10),
          Json().u("time_us", pwr.time_us).u("voltage_raw", pwr.voltage_raw)
              .i("current_raw", pwr.current_raw).i("soc_raw", pwr.soc_raw)
              .done(),
          "POWERData v1 (pre-#850); absent rail currents decode as absent, not 0");

    // NonSensor length ladder: 43 (base) / 44 (+apogee_flags) / 48
    // (+sensor_health) / 50 (+ekf_ticks).  Each shorter form is a faithful
    // prefix of the append-only struct.
    const auto ns = canonicalNonSensor();
    const auto nsFull = bytesOf(ns);
    for (size_t len : {size_t{43}, size_t{44}, size_t{48}, size_t{50}}) {
        char name[32];
        std::snprintf(name, sizeof(name), "nonsensor_%zu.bin", len);
        b.add("logframes", name, prefix(nsFull, len), nonSensorSidecar(ns, len),
              "NonSensorData ladder, msg 0xA5; absent tail fields decode as absent, not 0");
    }

    // OutStatusQuery version ladder (format_version byte at offset 9):
    // v3 = 26 B, v4 = 28 B (+iis2mdc rot), v5 = 41 B (+guidance echo tail),
    // v6 = 42 B (+mag_type).
    const auto sq = canonicalStatusQuery();
    auto sqFull = bytesOf(sq);
    b.add("logframes", "statusquery_v6_42.bin", sqFull,
          statusQuerySidecar(sq, 42, sq.format_version),
          "OutStatusQueryData v6 (+mag_type), msg 0xA0");
    auto sqV5 = prefix(sqFull, 41); sqV5[9] = 5;
    b.add("logframes", "statusquery_v5_41.bin", sqV5,
          statusQuerySidecar(sq, 41, 5),
          "OutStatusQueryData truncated to v5 (guidance echo tail, no mag_type)");
    auto sqV4 = prefix(sqFull, 28); sqV4[9] = 4;
    b.add("logframes", "statusquery_v4_28.bin", sqV4, statusQuerySidecar(sq, 28, 4),
          "OutStatusQueryData truncated to v4 (+IIS2MDC rotation, no guidance echo)");
    auto sqV3 = prefix(sqFull, 26); sqV3[9] = 3;
    b.add("logframes", "statusquery_v3_26.bin", sqV3, statusQuerySidecar(sq, 26, 3),
          "OutStatusQueryData truncated to v3 (b2r orientation, no IIS rotation)");

    // FlightSettings version ladder (version byte at offset 4):
    // v1 = 188, v2 = 200 (+b2r), v3/v4 = 208 (+fin cal; v4 = semantics only),
    // v5 = 210 (+imu rate), v6 = 219 (+flown guidance target).
    const auto fs = canonicalFlightSettings();
    const auto fsFull = bytesOf(fs);
    b.add("logframes", "flightsettings_v6_219.bin", fsFull,
          flightSettingsSidecar(fs, 219, fs.version),
          "FlightSettingsData v6, msg 0xE1; pyro sub-struct pinned by cmd34 fixture");
    const struct { size_t len; uint8_t ver; } fsLadder[] = {
        {188, 1}, {200, 2}, {208, 3}, {210, 5}};
    for (const auto& lv : fsLadder) {
        auto img = prefix(fsFull, lv.len);
        img[4] = lv.ver;
        char name[40];
        std::snprintf(name, sizeof(name), "flightsettings_v%u_%zu.bin", lv.ver, lv.len);
        b.add("logframes", name, img, flightSettingsSidecar(fs, lv.len, lv.ver),
              "FlightSettingsData version-ladder truncation of the v6 image");
    }
}

void buildFramed(Builder& b) {
    // A miniature but representative .bin log stream.
    const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> good = {
        {GNSS_MSG, bytesOf(canonicalGnss())},
        {ISM6HG256_MSG, bytesOf(canonicalImu())},
        {BMP585_MSG, bytesOf(canonicalBaro())},
        {MMC5983MA_MSG, bytesOf(canonicalMmc())},
        {NON_SENSOR_MSG, bytesOf(canonicalNonSensor())},
        {POWER_MSG, bytesOf(canonicalPower())},
        {IIS2MDC_MSG, bytesOf(canonicalIis())},
        {FLIGHT_SETTINGS_MSG, bytesOf(canonicalFlightSettings())},
    };
    std::vector<uint8_t> stream;
    std::string types = "[", lens = "[";
    for (size_t i = 0; i < good.size(); ++i) {
        append(stream, frame(good[i].first, good[i].second));
        if (i) { types += ","; lens += ","; }
        types += std::to_string(good[i].first);
        lens += std::to_string(good[i].second.size());
    }
    types += "]"; lens += "]";
    b.add("framed", "stream_good.bin", stream,
          Json().u("frame_count", good.size())
              .raw("types", types).raw("payload_lens", lens)
              .done(),
          "8 valid frames: [AA 55 AA 55][type][len][payload][CRC16 poly 0x8001 init 0, BE on wire]");

    // Bad CRC in the middle: parser must skip it and resync on the next preamble.
    std::vector<uint8_t> bad;
    append(bad, frame(GNSS_MSG, bytesOf(canonicalGnss())));
    auto corrupt = frame(ISM6HG256_MSG, bytesOf(canonicalImu()));
    corrupt[corrupt.size() - 1] ^= 0xFF;
    append(bad, corrupt);
    append(bad, frame(POWER_MSG, bytesOf(canonicalPower())));
    b.add("framed", "stream_badcrc_resync.bin", bad,
          Json().u("valid_frame_count", 2)
              .raw("valid_types", "[" + std::to_string(GNSS_MSG) + "," + std::to_string(POWER_MSG) + "]")
              .u("corrupt_frame_count", 1)
              .done(),
          "middle frame's CRC LSB flipped: skip + resync, do not abort");

    // Length-byte corruption: discriminates the documented WHOLE-FRAME-SKIP
    // resync from advance-by-one.  The ISM6 frame's len byte is inflated
    // 22→40, so the CRC (computed over the claimed 40-byte span) fails and
    // the skip lands 18 bytes past the frame's real end — swallowing the
    // POWER frame exactly and resuming at the IIS2MDC preamble.  An
    // advance-by-one parser would recover POWER (3 frames); iOS semantics
    // yield [GNSS, IIS2MDC] only.
    {
        std::vector<uint8_t> badlen;
        append(badlen, frame(GNSS_MSG, bytesOf(canonicalGnss())));
        auto inflated = frame(ISM6HG256_MSG, bytesOf(canonicalImu()));
        inflated[5] = 40;  // len byte: claimed 40, real payload 22
        append(badlen, inflated);
        append(badlen, frame(POWER_MSG, bytesOf(canonicalPower())));
        append(badlen, frame(IIS2MDC_MSG, bytesOf(canonicalIis())));
        b.add("framed", "stream_badlen_overshoot.bin", badlen,
              Json().u("valid_frame_count", 2)
                  .raw("valid_types", "[" + std::to_string(GNSS_MSG) + "," +
                                          std::to_string(IIS2MDC_MSG) + "]")
                  .done(),
              "inflated len byte: whole-frame-skip resync overshoots the next frame "
              "(iOS-accepted behavior); advance-by-one would recover it — must not");
    }

    // Truncated tail: parser must stop cleanly, keeping earlier frames.
    std::vector<uint8_t> trunc;
    append(trunc, frame(GNSS_MSG, bytesOf(canonicalGnss())));
    auto cut = frame(BMP585_MSG, bytesOf(canonicalBaro()));
    cut.resize(cut.size() - 6);
    append(trunc, cut);
    b.add("framed", "stream_truncated_tail.bin", trunc,
          Json().u("valid_frame_count", 1)
              .raw("valid_types", "[" + std::to_string(GNSS_MSG) + "]")
              .b("ends_truncated", true)
              .done(),
          "trailing frame cut mid-payload: stop, no throw, no resync loop");
}

void buildFileops(Builder& b) {
    // file_ops frames INCLUDE the leading discriminator byte — this is the
    // full-frame convention that resolves the 26-vs-27 0xCD ambiguity.

    // 0xAA frequency-scan result: [0xAA][start_mhz f32][step_khz f32][n u8][rssi i8 x n]
    std::vector<uint8_t> scan{0xAA};
    appendF32(scan, 903.0f);
    appendF32(scan, 200.0f);
    appendU8(scan, 5);
    for (int8_t r : {int8_t{-95}, int8_t{-102}, int8_t{-88}, int8_t{-120}, int8_t{-77}})
        appendI8(scan, r);
    b.add("fileops", "scan_0xAA_5samples.bin", scan,
          Json().f("start_mhz", 903.0f).f("step_khz", 200.0f).u("n", 5)
              .raw("rssi_dbm", intArray({-95, -102, -88, -120, -77}))
              .done(),
          "freq-scan result frame; start f32 sits at unaligned offset 1");

    // 0xCA mag-cal status ladder: struct sizes 22 / 26 / 32 / 36 by fw era.
    // Top rung is sizeof-derived so appending a field renames it (len40, ...)
    // — the freshness test then fails on the missing new file AND the orphaned
    // old one, forcing a deliberate ladder extension.
    const auto mc = canonicalMagCal();
    const auto mcFull = bytesOf(mc);
    for (size_t len : {size_t{22}, size_t{26}, size_t{32}, sizeof(MagCalStatusData)}) {
        std::vector<uint8_t> f{0xCA};
        append(f, prefix(mcFull, len));
        char name[40];
        std::snprintf(name, sizeof(name), "magcal_0xCA_len%zu.bin", len);
        b.add("fileops", name, f, magCalSidecar(mc, len),
              "MagCalStatusData ladder; frame length = 1 + struct bytes");
    }

    // 0xCB sensor-cal status.
    const auto sc = canonicalSensorCal();
    std::vector<uint8_t> scF{0xCB};
    append(scF, bytesOf(sc));
    b.add("fileops", "sensorcal_0xCB_19.bin", scF,
          Json().u("valid", sc.valid)
              .i("gyro_x", sc.gyro_x).i("gyro_y", sc.gyro_y).i("gyro_z", sc.gyro_z)
              .f("hg_x", sc.hg_x).f("hg_y", sc.hg_y).f("hg_z", sc.hg_z)
              .done(),
          "SensorCalStatusData behind 0xCB");

    // 0xCC rocket storage stats.
    const auto rs = canonicalRocketStorage();
    std::vector<uint8_t> rsF{0xCC};
    append(rsF, bytesOf(rs));
    b.add("fileops", "storage_rocket_0xCC.bin", rsF,
          Json().u("flight_region_blocks", rs.flight_region_blocks)
              .u("used_blocks", rs.used_blocks).u("free_blocks", rs.free_blocks)
              .u("bad_blocks", rs.bad_blocks).u("system_blocks", rs.system_blocks)
              .u("flight_count", rs.flight_count).u("block_size_kb", rs.block_size_kb)
              .u("flags", rs.flags)
              .done(),
          "RocketStorageStatsData behind 0xCC; flags = initialized|auto_evicted");

    // 0xCD base-station storage stats (u64 fields).
    const auto bs = canonicalBsStorage();
    std::vector<uint8_t> bsF{0xCD};
    append(bsF, bytesOf(bs));
    b.add("fileops", "storage_bs_0xCD.bin", bsF,
          Json().u("total_bytes", bs.total_bytes).u("used_bytes", bs.used_bytes)
              .u("free_bytes", bs.free_bytes).u("backend", bs.backend).u("flags", bs.flags)
              .done(),
          "BaseStationStorageStatsData behind 0xCD; frame = 27 B (1 + 26-B struct)");
}

// App→device command payloads: [cmd u8][payload], byte-exact.  Struct-backed
// payloads come straight from the header; the documented hand-packed layouts
// (cmd 5/9/10/50/60/70) are packed here and pinned by the freshness test.
void buildCommands(Builder& b) {
    auto cmd = [](uint8_t id, const std::vector<uint8_t>& payload) {
        std::vector<uint8_t> f{id};
        auto out = f;
        out.insert(out.end(), payload.begin(), payload.end());
        return out;
    };

    {
        // BLE cmd-5 wire is [mass_g][thrust_n][burn_s][descent_mps]: same field
        // order as SimConfigData but the first float is GRAMS — the OC relay
        // divides by 1000 into SimConfigData.mass_kg before forwarding to the
        // FC (out_computer main.cpp, BLE + LoRa handlers).  Do NOT bytesOf()
        // the struct here; that would pin a kg wire unit no firmware accepts.
        std::vector<uint8_t> p;
        appendF32(p, 595.0f);   // mass_g (firmware sim sees 0.595 kg)
        appendF32(p, 80.0f);    // thrust_n
        appendF32(p, 2.5f);     // burn_time_s
        appendF32(p, 15.0f);    // descent_rate_mps
        b.add("commands", "cmd05_simconfig_16.bin", cmd(5, p),
              Json().u("cmd", 5).f("mass_g", 595.0f).f("thrust_n", 80.0f)
                  .f("burn_time_s", 2.5f).f("descent_rate_mps", 15.0f).done(),
              "sim config; mass is GRAMS on the wire (OC /1000 -> SimConfigData.mass_kg)");
    }

    {
        std::vector<uint8_t> p;
        appendU16(p, 2026);
        for (uint8_t v : {7, 23, 14, 5, 9}) appendU8(p, v);
        b.add("commands", "cmd09_timesync.bin", cmd(9, p),
              Json().u("cmd", 9).u("year", 2026).u("month", 7).u("day", 23)
                  .u("hour", 14).u("minute", 5).u("second", 9).done(),
              "time sync, UTC; hand-packed [year u16][mo][d][h][m][s]");
    }

    {
        std::vector<uint8_t> p;
        appendF32(p, 903.0f);   // freq MHz
        appendF32(p, 250.0f);   // bw kHz
        appendU8(p, 9);         // sf
        appendU8(p, 5);         // cr
        appendI8(p, -3);        // tx power dBm — negative pins i8 bitPattern
        b.add("commands", "cmd10_lora_cfg.bin", cmd(10, p),
              Json().u("cmd", 10).f("freq_mhz", 903.0f).f("bw_khz", 250.0f)
                  .u("sf", 9).u("cr", 5).i("tx_pwr_dbm", -3).done(),
              "LoRa RF config; hand-packed [f32][f32][u8][u8][i8]");
    }

    {
        ServoConfigData s{};
        s.bias_us[0] = -120; s.bias_us[1] = 35; s.bias_us[2] = 0; s.bias_us[3] = 88;
        s.hz = 333; s.min_us = 1000; s.max_us = 2000;
        s.fin_min_deg = -12.5f; s.fin_max_deg = 12.5f;
        b.add("commands", "cmd12_servo_22.bin", cmd(12, bytesOf(s)),
              Json().u("cmd", 12).raw("bias_us", intArray({-120, 35, 0, 88}))
                  .i("hz", 333).i("min_us", 1000).i("max_us", 2000)
                  .f("fin_min_deg", -12.5f).f("fin_max_deg", 12.5f).done(),
              "ServoConfigData (#267 22-B form)");
    }

    b.add("commands", "cmd13_pid_20.bin",
          cmd(13, bytesOf(PIDConfigData{0.8f, 0.15f, 0.05f, -15.0f, 15.0f})),
          Json().u("cmd", 13).f("kp", 0.8f).f("ki", 0.15f).f("kd", 0.05f)
              .f("min_cmd", -15.0f).f("max_cmd", 15.0f).done(),
          "PIDConfigData");

    b.add("commands", "cmd24_servotest_8.bin",
          cmd(24, bytesOf(ServoTestAnglesData{{-2000, 1500, 0, 250}})),
          Json().u("cmd", 24).raw("angle_cdeg", intArray({-2000, 1500, 0, 250})).done(),
          "ServoTestAnglesData, centi-degrees");

    {
        const auto rp = canonicalRollProfile();
        b.add("commands", "cmd26_rollprofile_76.bin", cmd(26, bytesOf(rp)),
              Json().u("cmd", 26).u("num_waypoints", 3)
                  .raw("waypoints",
                       "[{\"time_s\": 0.5, \"angle_deg\": 0, \"mode\": 0},"
                       "{\"time_s\": 2, \"angle_deg\": 180, \"mode\": 0},"
                       "{\"time_s\": 4, \"angle_deg\": -90, \"mode\": 0}]")
                  .done(),
              "RollProfileData: [n u8][3 pad][8 x {t f32, a f32, mode u8}]; unused slots zeroed");
    }

    {
        std::vector<uint8_t> p;
        appendF64(p, 37.7749);
        appendF64(p, -122.4194);
        appendF32(p, 250.0f);
        b.add("commands", "cmd28_guidpoint_20.bin", cmd(28, p),
              Json().u("cmd", 28).d("lat_deg", 37.7749).d("lon_deg", -122.4194)
                  .f("alt_m", 250.0f).done(),
              "GuidancePointData {lat f64, lon f64, alt f32} — only f64 fields on the wire");
    }

    {
        RollControlConfigData r{};
        r.use_angle_control = 1; r._pad = 0; r.roll_delay_ms = 1200;
        r.kp_angle_rate_cap_dps = 120.0f; r.kp_angle = 2.5f;
        r.integral_sep_threshold_dps = -1.0f;  // sentinel: keep firmware default
        b.add("commands", "cmd31_rollctl_16.bin", cmd(31, bytesOf(r)),
              Json().u("cmd", 31).u("use_angle_control", 1).u("roll_delay_ms", 1200)
                  .f("kp_angle_rate_cap_dps", 120.0f).f("kp_angle", 2.5f)
                  .f("integral_sep_threshold_dps", -1.0f).done(),
              "RollControlConfigData; iwind < 0 = keep-firmware-default sentinel (#253)");
    }

    b.add("commands", "cmd33_camera_1.bin", cmd(33, bytesOf(CameraConfigData{2})),
          Json().u("cmd", 33).u("camera_type", 2).done(), "CameraConfigData (2 = RunCam)");

    b.add("commands", "cmd34_pyro_24.bin", cmd(34, bytesOf(canonicalPyro())),
          Json().u("cmd", 34)
              .raw("channels",
                   "[{\"enabled\": 1, \"mode\": 0, \"value\": 2.5},"
                   "{\"enabled\": 1, \"mode\": 1, \"value\": 150},"
                   "{\"enabled\": 0, \"mode\": 0, \"value\": 0},"
                   "{\"enabled\": 1, \"mode\": 0, \"value\": 8.75}]")
              .done(),
          "PyroConfigData: 4 x {en u8, mode u8, value f32}");

    b.add("commands", "cmd40_setname.bin", cmd(40, strBytes("RollyPolly III")),
          Json().u("cmd", 40).str("name", "RollyPolly III").u("payload_len", 14).done(),
          "set unit name; payload = raw UTF-8, firmware rejects > 20 bytes");
    b.add("commands", "cmd41_setnid.bin", cmd(41, {0x2A}),
          Json().u("cmd", 41).u("nid", 42).done(), "set network id");
    b.add("commands", "cmd42_setrid.bin", cmd(42, {7}),
          Json().u("cmd", 42).u("rid", 7).done(), "set rocket id (1..254)");
    b.add("commands", "cmd45_setfocus.bin", cmd(45, {7}),
          Json().u("cmd", 45).u("rid", 7).done(),
          "BS sticky focus (#390); 0 = auto");

    {
        // cmd 50 to a BS = relay envelope: [50][target_rid][inner cmd + payload].
        // Inner payload here is the cmd24 servo test (8 B) — total inner 9 B,
        // under the OC's 26-B relay RX cap.
        std::vector<uint8_t> inner = cmd(24, bytesOf(ServoTestAnglesData{{-2000, 1500, 0, 250}}));
        std::vector<uint8_t> p{7};
        append(p, inner);
        b.add("commands", "cmd50_relay_wrapped.bin", cmd(50, p),
              Json().u("cmd", 50).u("target_rid", 7).u("inner_cmd", 24)
                  .u("inner_payload_len", 8).done(),
              "BS relay envelope; SAME number is mag-cal start on a rocket link (dual namespace)");
    }

    {
        MagCalApplyData m{-321, 456, -789, 48.2f, 3.7f};
        b.add("commands", "cmd55_magcal_apply_14.bin", cmd(55, bytesOf(m)),
              Json().u("cmd", 55).i("cx", -321).i("cy", 456).i("cz", -789)
                  .f("R_uT", 48.2f).f("res_uT", 3.7f).done(),
              "MagCalApplyData");
    }

    {
        std::vector<uint8_t> p;
        appendF32(p, 902.0f);
        appendF32(p, 928.0f);
        appendU16(p, 200);
        appendU16(p, 150);
        b.add("commands", "cmd60_scan_12.bin", cmd(60, p),
              Json().u("cmd", 60).f("start_mhz", 902.0f).f("stop_mhz", 928.0f)
                  .u("step_khz", 200).u("dwell_ms", 150).done(),
              "freq scan request; hand-packed [f32][f32][u16][u16]");
    }

    {
        SensorCalApplyData s{-12, 34, -56, 0.25f, -0.5f, 9.81f};
        b.add("commands", "cmd62_sensorcal_18.bin", cmd(62, bytesOf(s)),
              Json().u("cmd", 62).i("gyro_x", -12).i("gyro_y", 34).i("gyro_z", -56)
                  .f("hg_x", 0.25f).f("hg_y", -0.5f).f("hg_z", 9.81f).done(),
              "SensorCalApplyData");
    }

    b.add("commands", "cmd64_imuorient_auto.bin", cmd(64, bytesOf(ImuOrientConfigData{IMU_ORIENT_AUTO})),
          Json().u("cmd", 64).u("setting", 255).done(),
          "ImuOrientConfigData; 0xFF = auto, else orientation code 0..23");

    {
        GuidanceConfigData g{};
        g.nav_gain = 3.5f; g.max_accel_mps2 = 30.0f; g.accel_to_fin_deg = 0.5f;
        g.max_fin_deg = 10.0f; g.min_speed_mps = 8.0f;
        g.target_e_m = 25.5f; g.target_n_m = -30.25f; g.target_alt_m = 250.0f;
        // enable/target_mode are adjacent u8s — keep them DISTINCT so a
        // field-swap decode is visible.  OVERHEAD ignores target_e/n; the
        // staged values stay for float coverage.
        g.coast_delay_ms = 500; g.enable = 1; g.target_mode = GUIDE_TARGET_OVERHEAD;
        g.kp_pos_per_s2 = 0.05f; g.kd_vel_per_s = 0.4f;
        g.guidance_law = GUIDE_LAW_STATION_KEEP;
        b.add("commands", "cmd65_guidance_45.bin", cmd(65, bytesOf(g)),
              Json().u("cmd", 65).f("nav_gain", 3.5f).f("max_accel_mps2", 30.0f)
                  .f("accel_to_fin_deg", 0.5f).f("max_fin_deg", 10.0f)
                  .f("min_speed_mps", 8.0f).f("target_e_m", 25.5f)
                  .f("target_n_m", -30.25f).f("target_alt_m", 250.0f)
                  .u("coast_delay_ms", 500).u("enable", 1).u("target_mode", 0)
                  .f("kp_pos_per_s2", 0.05f).f("kd_vel_per_s", 0.4f)
                  .u("guidance_law", 1).done(),
              "GuidanceConfigData: first 36 bytes FROZEN, append-only (#534); "
              "a 45-B-era FC rejects a 36-B frame — ship in lockstep");
    }

    {
        FinConfigData f{};
        f.azimuth_deg[0] = 0.0f; f.azimuth_deg[1] = 90.0f;
        f.azimuth_deg[2] = 180.0f; f.azimuth_deg[3] = 270.0f;
        f.reverse_mask = 0x05; f.roll_reverse_mask = 0x02;
        b.add("commands", "cmd66_fin_18.bin", cmd(66, bytesOf(f)),
              Json().u("cmd", 66)
                  .raw("azimuth_deg", "[0,90,180,270]")
                  .u("reverse_mask", 5).u("roll_reverse_mask", 2).done(),
              "FinConfigData");
    }

    b.add("commands", "cmd67_imurate_2.bin", cmd(67, bytesOf(ImuRateConfigData{1920})),
          Json().u("cmd", 67).u("rate_hz", 1920).done(),
          "ImuRateConfigData; valid values 960/1920/3840");

    {
        // OTA_BEGIN: [target u8][size u32 LE][sha256 32 B]; not a header struct
        // (lives in TR_BLE_To_APP.cpp) — layout pinned here.
        std::vector<uint8_t> p{1};  // target: 1 = FC via relay
        const uint32_t size = 1234567;
        p.push_back(size & 0xFF); p.push_back((size >> 8) & 0xFF);
        p.push_back((size >> 16) & 0xFF); p.push_back((size >> 24) & 0xFF);
        std::string sha;
        for (int i = 0; i < 32; ++i) {
            p.push_back(static_cast<uint8_t>(i));
            char h[3];
            std::snprintf(h, sizeof(h), "%02x", i);
            sha += h;
        }
        b.add("commands", "cmd70_otabegin_37.bin", cmd(70, p),
              Json().u("cmd", 70).u("target", 1).u("size", size).str("sha256_hex", sha).done(),
              "OTA_BEGIN; cmds 70-72 dispatch inside TR_BLE_To_APP.cpp, not main.cpp");
    }
}

// A miniature synthetic flight for the cross-platform bin→CSV golden
// (android-port plan Phase 1 exit gate).  Mini-device, ~750 frames, every
// value derived from integer arithmetic — NO floats/transcendentals in frame
// synthesis, so the byte image is identical on every host.  The profile
// deliberately exercises: statusQuery rotation config, the 4-s pre-launch
// trim (launch at 8 s → rows start at 4 s, dropping the 1.2–4 s pad wait),
// ground-pressure averaging (strictly-before-launch window), forward-fill,
// event-flag latching (launch 8 s, burnout 10 s, apogee 14 s, landed 19 s),
// and the master-apogee max-speed gate.
void buildCsvFlight(Builder& b) {
    std::vector<uint8_t> stream;
    // The stream's mag counts were synthesized under IIS2MDC semantics, so
    // its config frame must say IIS2MDC — the canonical's QMC5883P mag_type
    // (kept nonzero for the standalone v6 fixture) would rescale the CSV mag
    // columns and invalidate the committed app-generated goldens.
    auto csvQuery = canonicalStatusQuery();
    csvQuery.mag_type = MAG_TYPE_IIS2MDC;
    append(stream, frame(OUT_STATUS_QUERY, bytesOf(csvQuery)));   // t=1.0 s (rotation config)
    append(stream, frame(FLIGHT_SETTINGS_MSG, bytesOf(canonicalFlightSettings())));

    int frames = 2;
    for (int i = 0; i <= 188; ++i) {
        const uint32_t t_ms = 1200 + 100u * i;
        const uint32_t t_us = t_ms * 1000u;

        // Barometer every tick: pad wiggle → linear boost drop → descent rise.
        {
            BMP585Data d{};
            d.time_us = t_us;
            d.temp_q16 = 1540096 - i * 256;
            if (t_ms < 8000) {
                d.press_q6 = 6484800u + ((i % 5) - 2) * 64;
            } else if (t_ms < 14000) {
                d.press_q6 = 6484800u - (t_ms - 8000) * 64;
            } else {
                d.press_q6 = 6484800u - 384000u + (t_ms - 14000) * 60;
            }
            append(stream, frame(BMP585_MSG, bytesOf(d)));
            ++frames;
        }

        // IIS2MDC magnetometer every tick.
        {
            IIS2MDCData d{};
            d.time_us = t_us;
            d.mag_x = static_cast<int16_t>(-2000 + 10 * i);
            d.mag_y = static_cast<int16_t>(5 * i - 400);
            d.mag_z = 1234;
            append(stream, frame(IIS2MDC_MSG, bytesOf(d)));
            ++frames;
        }

        // IMU from 1.5 s — the row driver (one CSV row per IMU frame).
        if (t_ms >= 1500) {
            ISM6HG256Data d{};
            d.time_us = t_us;
            d.acc_low_raw = {static_cast<int16_t>(-1000 + 7 * i),
                             static_cast<int16_t>(500 - 3 * i),
                             static_cast<int16_t>((t_ms >= 8000 && t_ms < 10000) ? 26000 : 16000)};
            d.acc_high_raw = {static_cast<int16_t>(-80 + i),
                              static_cast<int16_t>(60 - i), 120};
            d.gyro_raw = {static_cast<int16_t>((i * 37) % 4001 - 2000), 1000,
                          static_cast<int16_t>((i * 53) % 3001 - 1500)};
            append(stream, frame(ISM6HG256_MSG, bytesOf(d)));
            ++frames;
        }

        // NonSensor from 1.3 s: quat keyframes, event flags, EKF velocities.
        if (t_ms >= 1300) {
            static const int16_t quatTable[8][4] = {
                {10000, 0, 0, 0}, {9239, 3827, 0, 0}, {7071, 7071, 0, 0},
                {3827, 9239, 0, 0}, {0, 10000, 0, 0}, {-3827, 9239, 0, 0},
                {-7071, 7071, 0, 0}, {-9239, 3827, 0, 0}};
            const int16_t* q = quatTable[(i / 4) % 8];
            NonSensorData d{};
            d.time_us = t_us;
            d.q0 = q[0]; d.q1 = q[1]; d.q2 = q[2]; d.q3 = q[3];
            d.roll_cmd = static_cast<int16_t>((i * 25) % 9000 - 4500);
            d.e_pos = 100 * i; d.n_pos = -50 * i;
            d.u_pos = (t_ms >= 8000) ? (t_ms - 8000) * 8 : 0;
            d.e_vel = 100; d.n_vel = -200;
            if (t_ms < 8000)       d.u_vel = 0;
            else if (t_ms < 10000) d.u_vel = (t_ms - 8000) * 8;    // boost → 16000 cm/s
            else if (t_ms < 14000) d.u_vel = 16000 - (t_ms - 10000) * 4;  // coast → 0
            else                   d.u_vel = -static_cast<int32_t>(t_ms - 14000) * 2;
            uint8_t flags = 0;
            if (t_ms >= 19000) flags |= 0x01;   // alt_landed
            if (t_ms >= 14000) flags |= 0x02;   // alt_apogee
            if (t_ms >= 13900) flags |= 0x04;   // vel_u_apogee
            if (t_ms >= 8000)  flags |= 0x08;   // launch
            if (t_ms >= 10000) flags |= 0x10;   // burnout (#196 latch source)
            d.flags = flags;
            d.rocket_state = (t_ms < 8000) ? 3 : (t_ms < 14000) ? 5 : (t_ms < 19000) ? 6 : 7;
            d.baro_alt_rate_dmps = static_cast<int16_t>(d.u_vel / 10);
            // PSF bits are interleaved per channel (cont, fired), so the
            // pre-apogee value is ch1+ch2 CONTINUITY and nothing fired.
            d.pyro_status = (t_ms >= 14000) ? 0x0F : 0x05;
            uint8_t af = 0;
            if (t_ms >= 14100) af |= 0x01;      // gps apogee (per-detector)
            if (t_ms >= 14000) af |= 0x04;      // MASTER voted apogee (max-speed gate)
            d.apogee_flags = af;
            d.sensor_health = 0x00500541u;
            d.ekf_ticks = static_cast<uint16_t>((i * 13) & 0xFFFF);
            append(stream, frame(NON_SENSOR_MSG, bytesOf(d)));
            ++frames;
        }

        // GNSS every 200 ms from 2 s.
        if (t_ms >= 2000 && (i % 2) == 0) {
            GNSSData d = canonicalGnss();
            d.time_us = t_us;
            d.lat_e7 = 377749000 + 10 * i;
            d.lon_e7 = -1224194000 - 10 * i;
            d.alt_mm = 123456 + ((t_ms >= 8000 && t_ms < 14000) ? (t_ms - 8000) * 80
                                 : (t_ms >= 14000) ? 480000 - (t_ms - 14000) * 40 : 0);
            d.vel_u_mmps = (t_ms >= 8000 && t_ms < 10000) ? (t_ms - 8000) * 80 : -900;
            append(stream, frame(GNSS_MSG, bytesOf(d)));
            ++frames;
        }

        // Power every 500 ms from 2 s.
        if (t_ms >= 2000 && (i % 5) == 0) {
            POWERData d{};
            d.time_us = t_us;
            d.voltage_raw = static_cast<uint16_t>(27000 - 10 * i);
            d.current_raw = static_cast<int16_t>(-1234 + 20 * i);
            d.soc_raw = static_cast<int16_t>(15000 - 40 * i);
            append(stream, frame(POWER_MSG, bytesOf(d)));
            ++frames;
        }
    }

    b.add("csv", "tiny_flight.bin", stream,
          Json().u("frame_count", frames)
              .u("launch_ms", 8000).u("burnout_ms", 10000)
              .u("apogee_ms", 14000).u("landed_ms", 19000)
              .u("trim_start_ms", 4000)   // max(first IMU 1.5 s, launch − 4 s)
              .u("expected_rows", 161)    // IMU frames with t >= 4.0 s
              .done(),
          "synthetic mini-device flight for the bin→CSV golden; app-generated "
          "CSV/summary goldens live in tests_cpp/fixtures/csv_golden/ (outside "
          "wire/ — they are app output, not emitter output)");
}


// --------------------------------------------------- base-station log golden ---
// A short base-station binary log built with the REAL packers, so the Python,
// Kotlin and Swift readers all test against bytes the firmware itself produced
// rather than against each other's idea of the layout.
//
// Deliberately exercises the things that are easy to get wrong: both frame
// types on the real 5:1 schedule, a slow frame that must not blank the
// position, a fast frame that must not blank the battery, an unknown-RSSI
// sentinel, a sequence gap, and an event record interleaved with telemetry.
void buildBsLogGolden(Builder& b) {
    std::vector<uint8_t> stream;
    const uint8_t magic[8] = {'T', 'R', 'B', 'S', 'L', 'O', 'G',
                              BS_LOG_FORMAT_VERSION};
    append(stream, std::vector<uint8_t>(magic, magic + sizeof(magic)));

    SensorConverter conv;
    LoRaDataSI si{};
    si.network_id = 0;
    si.rocket_id = 1;
    si.next_channel_idx = LORA_NEXT_CH_NO_HOP;
    si.rocket_state = 2;            // PRELAUNCH
    si.launch_flag = false;
    si.num_sats = 11;
    si.pdop = 2.0f;
    si.horizontal_accuracy = 5.0f;
    si.ecef_x = 1113194.0;  si.ecef_y = -4813733.0;  si.ecef_z = 3985732.0;
    si.acc_x = 0.1f; si.acc_y = -0.2f; si.acc_z = 9.8f;
    si.gyro_x = 1.5f; si.gyro_y = -2.5f; si.gyro_z = 0.5f;
    si.q0 = 1.0f; si.q1 = 0.0f; si.q2 = 0.0f; si.q3 = 0.0f;
    si.pressure_alt = 100.0f;
    si.altitude_rate = 0.0f;
    si.vel_e = 1.0f; si.vel_n = -2.0f; si.vel_u = 3.0f;
    si.sensor_health = 0x00015555u;
    si.max_alt = 120.0f;
    si.max_speed = 12.0f;
    si.temp = 21.5f;
    si.voltage = 7.90f;
    si.current = -450.0f;
    si.soc = 79.0f;
    si.cam_current = 1.48f;
    si.servo_current = 0.34f;
    si.burnout_detected = false;
    si.imu_orient_code = 0;
    si.imu_orient_mode = 1;

    uint32_t frames = 0;
    // seq 3 is skipped on purpose so a reader has a gap to compute.
    const uint16_t seqs[] = {0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    for (uint16_t seq : seqs) {
        si.seq = seq;
        si.pressure_alt = 100.0f + static_cast<float>(seq);   // moves every frame
        si.max_alt      = 120.0f + static_cast<float>(seq);   // slow-frame only

        uint8_t frame_bytes[SIZE_OF_LORA_FAST] = {0};
        size_t  frame_len = 0;
        if (loraFrameTypeForSlot(seq) == LORA_FRAME_SLOW) {
            conv.packLoRaSlowBytes(si, frame_bytes);
            frame_len = SIZE_OF_LORA_SLOW;
        } else {
            conv.packLoRaFastBytes(si, frame_bytes);
            frame_len = SIZE_OF_LORA_FAST;
        }

        BsLoRaRxHeader hdr{};
        hdr.time_ms = 500u * static_cast<uint32_t>(seq);
        // One record carries the "no reading" sentinel so readers prove they
        // render it as absent rather than as a plausible 0 dBm.
        hdr.rssi_dbm_x10 = (seq == 2) ? BS_RSSI_UNKNOWN : static_cast<int16_t>(-410);
        hdr.snr_db_x10   = (seq == 2) ? BS_RSSI_UNKNOWN : static_cast<int16_t>(120);
        hdr.rx_freq_hz   = 915000000u;

        std::vector<uint8_t> payload;
        append(payload, bytesOf(hdr));
        append(payload, std::vector<uint8_t>(frame_bytes, frame_bytes + frame_len));
        append(stream, frame(BS_LORA_RX_MSG, payload));
        ++frames;
    }

    // One interleaved event record.
    {
        const char* text = "hop start";
        BsEventHeader eh{};
        eh.time_ms = 3200;
        eh.rx_freq_hz = 915500000u;
        eh.text_len = static_cast<uint8_t>(strlen(text));
        std::vector<uint8_t> payload;
        append(payload, bytesOf(eh));
        append(payload, std::vector<uint8_t>(text, text + eh.text_len));
        append(stream, frame(BS_EVENT_MSG, payload));
    }

    b.add("csv", "bs_tiny.bin", stream,
          Json().u("rx_records", frames).u("event_records", 1)
              .u("fast_frames", 10).u("slow_frames", 2)
              .u("skipped_seq", 3)
              .done(),
          "base-station binary log built with the real packers; the CSV golden "
          "generated from it lives in tests_cpp/fixtures/csv_golden/ (app "
          "output, not emitter output)");
}

}  // namespace

FixtureMap generate() {
    if (!hostIsLittleEndian()) {
        // Struct byte images would not be the wire format on a BE host.
        return {};
    }
    Builder b;
    buildLogframes(b);
    buildBsLogGolden(b);
    buildFramed(b);
    buildFileops(b);
    buildCommands(b);
    buildCsvFlight(b);

    // Manifest last: lists every fixture (sidecars excluded — they pair 1:1).
    const std::string manifest =
        "{\n \"format\": 1,\n \"note\": \"generated by wire_fixture_gen — do not edit; "
        "regen via cmake --build <build> --target regen-wire-fixtures\",\n \"fixtures\": [" +
        b.manifest + "\n ]\n}\n";
    b.out["manifest.json"] = strBytes(manifest);
    return b.out;
}

}  // namespace tr_wire_fixtures

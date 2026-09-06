#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <RocketComputerTypes.h>

// The two-MCU boards move data between the flight computer and the out
// computer over I2S (telemetry), I2C (commands) and the NAND (logging via the
// OC). On the mini both roles run in one process, and this header is what
// remains of that link: the same message frames, without the wires.
//
// Three paths, replacing three transports:
//   1. logFrame()  — flight side writes the standard AA55AA55|type|len|
//      payload|CRC16 frames straight into the log ring. The NAND stream
//      stays byte-identical to the OC's, so every existing log consumer
//      (BLE download, flight-report tooling) parses mini logs unchanged.
//   2. telem       — flight side updates the latest-state cache in place;
//      comms side snapshots it for LoRa/BLE. Replaces the OC's I2S parser
//      cache. Guarded by telem_mux; keep critical sections to plain copies.
//   3. cmd_queue   — comms side enqueues the SAME message-type frames it
//      used to relay over I2C (SIM_CONFIG_MSG, PYRO_CONFIG_MSG, MAG_CAL_*,
//      ...); the flight loop drains and dispatches once per tick. Replaces
//      the OUT_STATUS_QUERY / pending-command dance.
namespace mini_link {

// ---- 3. comms → flight commands ----
struct CmdFrame {
    uint8_t type;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD];
};

// Depth 8: commands are human-initiated (BLE taps, uplink packets); the
// flight loop drains the whole queue every tick, so depth only covers a
// burst arriving inside one tick.
constexpr UBaseType_t CMD_QUEUE_DEPTH = 8;
extern QueueHandle_t cmd_queue;

// Enqueue for the flight side. false = queue full (caller logs; nothing on
// this path is allowed to block).
bool sendCommand(uint8_t type, const uint8_t* payload, uint8_t len);

// ---- 2. flight → comms latest-state cache ----
struct TelemState {
    ISM6HG256Data imu;
    BMP585Data baro;
    IIS2MDCData mag;
    GNSSData gnss;
    NonSensorData nonsensor;          // EKF state, flags, rocket state, pyro
    MagCalStatusData mag_cal;
    SensorCalStatusData sensor_cal;
    // esp_timer micros of the last update of each group; 0 = never. The
    // comms side uses these for staleness exactly where the OC used
    // last-frame-received times.
    uint32_t imu_update_us;
    uint32_t baro_update_us;
    uint32_t mag_update_us;
    uint32_t gnss_update_us;
    uint32_t nonsensor_update_us;
    uint32_t mag_cal_update_us;
    uint32_t sensor_cal_update_us;
};
extern TelemState telem;
extern portMUX_TYPE telem_mux;

// ---- 1. flight → log ----
// Packs the standard frame (TR_I2C_Interface::packMessage) and enqueues it
// on the TR_LogToFlash ring. Drops silently (returns false) until
// enableLogSink() — the logger only exists once the peripheral rail is up.
bool logFrame(uint8_t type, const uint8_t* payload, uint8_t len);
void enableLogSink(bool on);

// ---- lifecycle ----
// Creates cmd_queue. Call once from app_main before either side starts.
void init();

// True on a boot whose power-on was triggered by the ACTIVE-restore policy
// (main.cpp found the persisted ACTIVE flag set — the previous session ended
// in an unexpected reset). Set by main.cpp BEFORE the automatic power-on
// request; never set on a manual, user-commanded power-on. The flight side's
// snapshot recovery keys on this: a mid-flight snapshot may only be restored
// on the reboot that interrupted that flight, not on a later manual
// power-up with a stale interrupted flight still on the chip.
extern bool active_restore_boot;

// #1176: the flight side reached LANDED, so a flight genuinely ended. Clears
// the ACTIVE-restore RETRY COUNTER (not the ACTIVE flag itself — an unexpected
// reset after landing should still re-enter ACTIVE and keep a downed rocket's
// tracker alive).
//
// This exists because deleting the old "60 s of healthy ACTIVE forgives the
// budget" rule removed the counter's ONLY refill. Every real pad session
// exceeds 60 s, so that rule silently guaranteed a full budget at every
// launch; without a replacement the counter would ratchet across battery
// reconnects and could refuse the one in-flight restore this issue exists to
// enable. `genuine` is false when the flight is ending because the recovery
// interlock REFUTED it — a wrongly restored flight on a bench must not refill
// the budget that bounds it.
void flightEnded(bool genuine);

// #1176: this reboot is DELIBERATE, so the next boot must not read it as a
// session that ended unexpectedly. Clears the ACTIVE flag and its retry
// counter. Recovery eligibility no longer consults the reset reason, so the
// deliberate-reboot paths have to say so for themselves — otherwise a
// post-OTA boot looks exactly like a power loss mid-flight.
void retireActiveFlag(const char* why);

// Command shutter: sendCommand() refuses (returns false) while closed.
// Closed at boot and during power-off (nothing should reach a flight side
// that doesn't exist / is being torn down); opened by main.cpp once the
// flight task is running. BLE/uplink handlers surface the refusal to the
// operator instead of silently queueing commands for a later session —
// a PYRO_FIRE_TEST tapped in IDLE must not fire at the next power-on.
void setCommandShutter(bool open);
bool commandShutterOpen();

}  // namespace mini_link

#pragma once

#include <stdint.h>

// Comms side (comms.cpp): the out_computer's job, in-process. Split into an
// always-on half (BLE, INA230, identity — alive in IDLE) and a rail-backed
// half (logger/flightlog on the NAND, LoRa radio) initialized on power-on.

// Always-on bring-up: NVS identity, BLE begin + advertising, INA230 on the
// shared I2C bus. Called once from app_main, in IDLE, rail down.
void comms_setup_idle();

// Rail-backed bring-up: logger + flightlog (NAND), LoRa direct backend
// (factory rendezvous, then NVS config). Called during the power-on
// transition after the rail settles and before flight_setup().
// Returns false if the NAND or radio failed — caller decides whether to
// continue degraded.
bool comms_setup_active();

// The comms task entry: core 0, prio 2, 12 KB stack. Services BLE commands,
// LoRa TX/RX + hopping, INA230 polling, logging control, telemetry — the
// oc_loop successor. Runs in both modes; rail-backed work is gated
// internally on the mode.
void commsTask(void* arg);

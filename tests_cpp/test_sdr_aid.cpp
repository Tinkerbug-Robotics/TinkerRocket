// Host tests for TinkerRocket's message 4002, the flight computer to the
// gnss-sdr-dev board (TR_GNSSReceiverSdr_Serial/SdrAid.cpp): the golden
// frames both repos pin, the stamp, the P1 bench script and the schedule that
// paces it, the aiding block the board appends to its 4001 epoch, with the
// frames it says it took, and the flight image's phase (flight_computer/main/
// sdr_phase_policy.h): its map from the state machine's flags and the IMU's
// ignition cue, its keeping of the board's clock line and its gating on it,
// and its pacing; and its attitude and rests beside the phase (flight_
// computer/main/sdr_aid_policy.h): body to ECEF at the board's own fix, the
// still windows and their schedule. The golden bytes come from the P1 spec's
// reference codec (v10link_codec.py); the board's
// receiver/tests/test_rtcm_fc.cpp pins the same ones.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "GnssClockRecord.h"
#include "GnssTimeSync.h"
#include "Rtcm3Msm.h"
#include "SdrAid.h"
#include "SdrRtcm.h"
#include "fixtures/sdr_rtcm_stream.h"
#include "sdr_phase_policy.h"   // projects/flight_computer/main: the flight image's phase
#include "sdr_aid_policy.h"     // and its attitude and rests

using namespace sdr_aid;

namespace {

using Bytes = std::vector<uint8_t>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kR2 = 0.7071067811865476;

// The same bytes as receiver/tests/test_rtcm_fc.cpp in the gnss repo (P1's
// p1_golden.json): change both together.

// A1: attitude, ordinary
const Bytes kA1 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x21, 0x09, 0x80, 0x30, 0xA3, 0x2D, 0xED, 0xBE,
    0x15, 0x41, 0xE3, 0xC3, 0x33, 0x3E, 0x8F, 0x61, 0xEB, 0x80, 0x00, 0x00,
    0x08, 0x03, 0xFB, 0x00, 0x03, 0x00, 0x03, 0x43, 0xC0, 0xBF, 0x7E, 0x39};
// A2: field ends: the week's last ms, sub_ns past -0.5 ms, seq 255; q0 < 0
// (sent negated), rates past +-128 rad/s and a half LSB (away from zero),
// sigmas past the field and NaN
const Bytes kA2 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x21, 0xFF, 0xFF, 0x90, 0x32, 0x0F, 0xFE, 0x00,
    0x00, 0x7F, 0xFF, 0xC2, 0x66, 0x60, 0x00, 0x00, 0x00, 0x03, 0x33, 0x37,
    0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x7F, 0xFF, 0xC0, 0xA0, 0xA7, 0x8C};
// A3: no attitude (valid 0): the fields 0 whatever they hold
const Bytes kA3 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x21, 0x09, 0x80, 0x00, 0x00, 0x00, 0x04, 0x00,
    0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0xA8, 0x3C};
// A4: a NaN in q: att_valid sent 0 (q and both sigmas 0); infinite rates saturate
const Bytes kA4 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x21, 0x09, 0x80, 0x30, 0xA3, 0x2F, 0x20, 0x00,
    0x00, 0x02, 0x53, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
    0xFF, 0xFE, 0x00, 0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0xCD, 0xC3, 0x68};
// N1: navigation state, ordinary
const Bytes kN1 = {
    0xD3, 0x00, 0x2E, 0xFA, 0x22, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0xC8, 0x00,
    0x00, 0x00, 0x23, 0xC0, 0xBF, 0xF6, 0xA6, 0xC8, 0xB5, 0x02, 0x38, 0xC3,
    0xD2, 0x25, 0xFE, 0x8E, 0x1B, 0xD3, 0xFF, 0xFE, 0xCB, 0x81, 0xEF, 0xD1,
    0x03, 0xFF, 0xD8, 0xEF, 0xC0, 0x3E, 0x80, 0x64, 0x00, 0x25, 0x80, 0x3E,
    0x80, 0xCE, 0xDC, 0x2D};
// N2: field ends: positions and velocities past their fields, halves away
// from zero, sigmas past the field, 0 (sent as 1), NaN and negative; at the
// V10's IMU
const Bytes kN2 = {
    0xD3, 0x00, 0x2E, 0xFA, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF,
    0xFF, 0xE0, 0x11, 0xD7, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00,
    0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x7F, 0xFF, 0xFF, 0xFF,
    0xC0, 0x99, 0x0F, 0xF3};
// N3: a position without a velocity (vel_valid 0: its fields 0)
const Bytes kN3 = {
    0xD3, 0x00, 0x2E, 0xFA, 0x22, 0x09, 0x80, 0x30, 0xA6, 0x87, 0x63, 0xFF,
    0xFF, 0xC0, 0x4C, 0x8E, 0x33, 0x24, 0x6B, 0x11, 0x33, 0x02, 0x5C, 0xCD,
    0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x7C, 0x0A, 0x09};
// N4: a NaN in the velocity: vel_valid sent 0 (its fields and sigmas 0); the
// position kept
const Bytes kN4 = {
    0xD3, 0x00, 0x2E, 0xFA, 0x22, 0x09, 0x80, 0x30, 0xA3, 0x2F, 0xE8, 0x00,
    0x00, 0x02, 0x93, 0x80, 0xBF, 0xF6, 0xA6, 0xC8, 0xB5, 0x02, 0x38, 0xC3,
    0xD2, 0x25, 0xFE, 0x8E, 0x1B, 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x80, 0x64, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x51, 0x5A, 0xE6};
// P1: phase, ordinary: the boost, its loops asked for, 2.5 s hold
const Bytes kP1 = {
    0xD3, 0x00, 0x10, 0xFA, 0x23, 0x09, 0x80, 0x30, 0xA3, 0x3D, 0x30, 0x00,
    0x00, 0x00, 0xCC, 0x50, 0x10, 0x41, 0xF4, 0xCD, 0x6C, 0xC3};
// P2: field ends: landed, canopy, since past 65.535 s, hold past 5.1 s,
// sub_ns at its top
const Bytes kP2 = {
    0xD3, 0x00, 0x10, 0xFA, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF,
    0xFF, 0xFF, 0xD7, 0xAF, 0xFF, 0xFF, 0xFC, 0xBD, 0x78, 0xE8};
// P3: no request (loops none, hold 0) in coast
const Bytes kP3 = {
    0xD3, 0x00, 0x10, 0xFA, 0x23, 0x09, 0x80, 0x30, 0xA3, 0xCA, 0x9A, 0x79,
    0x60, 0x01, 0xE3, 0x60, 0x5C, 0x30, 0x00, 0x41, 0xAD, 0x41};
// G1: geometry, ordinary: the script's lever arm and a 90 deg turn about +z,
// a patch along +z
const Bytes kG1 = {
    0xD3, 0x00, 0x1C, 0xFA, 0x24, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x4C, 0xC6, 0x80, 0x00, 0x0C, 0xFF, 0xDE, 0x00, 0x9C, 0x2D,
    0x41, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x41, 0x7C, 0x3A, 0xC1};
// G2: field ends: lever arms past +-32.767 m and a half mm (away from zero),
// r0 < 0 (sent negated), the antenna fields at their tops
const Bytes kG2 = {
    0xD3, 0x00, 0x1C, 0xFA, 0x24, 0xFF, 0xFF, 0x90, 0x32, 0x0F, 0xFE, 0x00,
    0x00, 0x7F, 0xF7, 0xFF, 0x80, 0x7F, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0x20,
    0x00, 0xE0, 0x00, 0x20, 0x00, 0xE0, 0x00, 0x3D, 0xC2, 0x4B};
// G3: nothing known (both valid bits 0)
const Bytes kG3 = {
    0xD3, 0x00, 0x1C, 0xFA, 0x24, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDA, 0x24, 0x43};
// G4: a NaN in the lever arm: lever_valid sent 0; the rotation kept
const Bytes kG4 = {
    0xD3, 0x00, 0x1C, 0xFA, 0x24, 0x09, 0x80, 0x30, 0xA3, 0x30, 0xB0, 0x00,
    0x00, 0x02, 0xD0, 0x46, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D,
    0x41, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x41, 0xED, 0x66, 0xD1};
// R1: at rest, ordinary: a still second, nearly 1 g along +z (FRD: down)
const Bytes kRest1 = {
    0xD3, 0x00, 0x17, 0xFA, 0x25, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x55, 0x83, 0xE8, 0x00, 0x07, 0xBF, 0xEE, 0x29, 0xE8, 0x1F,
    0x70, 0x50, 0x4F, 0xEB, 0x54};
// R2: field ends: the week's last ms, sub_ns at its bottom, seq 255; a
// window past 4.095 s, forces past +-52.4287 m/s^2 and a half LSB (away from
// zero), the sigma unknown
const Bytes kRest2 = {
    0xD3, 0x00, 0x17, 0xFA, 0x25, 0xFF, 0xFF, 0x90, 0x32, 0x0F, 0xFE, 0x00,
    0x00, 0x7F, 0xFF, 0x8F, 0xFF, 0x7F, 0xFF, 0xF8, 0x00, 0x01, 0xFF, 0xFF,
    0xFF, 0xF0, 0x88, 0xAC, 0xE6};
// R3: not still (valid 0): the force sent 0 whatever it holds, the sigma as
// it is (under its LSB: 1)
const Bytes kRest3 = {
    0xD3, 0x00, 0x17, 0xFA, 0x25, 0x09, 0x80, 0x30, 0xA3, 0x33, 0xD0, 0x00,
    0x00, 0x00, 0x95, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x01, 0x2D, 0x81};
// R4: a NaN in the force: valid sent 0 (the force 0), the sigma kept
const Bytes kRest4 = {
    0xD3, 0x00, 0x17, 0xFA, 0x25, 0x09, 0x80, 0x30, 0xA3, 0x7A, 0x20, 0x00,
    0x00, 0x00, 0xD5, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x30, 0xB1, 0x13, 0x76};

// A5 and R5: A1's and R1's inputs in the flight computer's IMU axes (frame
// 1: its board frame, before the mounting), as the flight image sends both
// since 2026-11-12 (the reference codec's imu_frame_goldens.py; the board's
// test_rtcm_fc.cpp pins the same bytes)
const Bytes kA5 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x21, 0x09, 0x80, 0x30, 0xA3, 0x2D, 0xED, 0xBE,
    0x15, 0x41, 0xE3, 0xD3, 0x33, 0x3E, 0x8F, 0x61, 0xEB, 0x80, 0x00, 0x00,
    0x08, 0x03, 0xFB, 0x00, 0x03, 0x00, 0x03, 0x43, 0xC0, 0x3B, 0x87, 0x49};
const Bytes kRest5 = {
    0xD3, 0x00, 0x17, 0xFA, 0x25, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x55, 0xC3, 0xE8, 0x00, 0x07, 0xBF, 0xEE, 0x29, 0xE8, 0x1F,
    0x70, 0x50, 0xF4, 0xC4, 0x3E};

// The script's frames (S + sub-type + _ + tow_ms): week 2432, sub_ns 0, src
// 0, test, tq 5, seq as each test says.
const Bytes kS1_204000000 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x21, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x0D, 0xC1, 0x5A, 0x3E, 0x47, 0xA2, 0x41, 0xCD, 0x86, 0xC0,
    0x01, 0xAC, 0x00, 0x00, 0x00, 0x00, 0x01, 0x4F, 0x00, 0xAB, 0x87, 0x0B};
const Bytes kS1_204005000 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x21, 0x09, 0x80, 0x30, 0xA3, 0x7A, 0x20, 0x00,
    0x00, 0x00, 0x4D, 0xC1, 0xC0, 0x6E, 0xB0, 0x41, 0x8A, 0x3D, 0x06, 0xD0,
    0x01, 0xAC, 0x00, 0x00, 0x00, 0x00, 0x01, 0x4F, 0x00, 0x49, 0x3D, 0xAD};
const Bytes kS1_204017250 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x21, 0x09, 0x80, 0x30, 0xA4, 0x39, 0x88, 0x00,
    0x00, 0x00, 0x8D, 0xC2, 0x30, 0x3F, 0xFF, 0x4F, 0x74, 0x6C, 0xB2, 0x40,
    0x01, 0xAC, 0x00, 0x00, 0x00, 0x00, 0x01, 0x4F, 0x00, 0x58, 0x0F, 0xB7};
const Bytes kS2_204000000 = {
    0xD3, 0x00, 0x2E, 0xFA, 0x22, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x0D, 0xCE, 0x33, 0x24, 0x6B, 0x11, 0x33, 0x02, 0x5C, 0xCD,
    0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0xBB, 0xFF, 0xFF, 0xA0,
    0xD0, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x7D, 0x00, 0x32, 0x00, 0x4B,
    0x00, 0xA5, 0x6F, 0xAE};
const Bytes kS2_204002500 = {
    0xD3, 0x00, 0x2E, 0xFA, 0x22, 0x09, 0x80, 0x30, 0xA3, 0x53, 0x10, 0x00,
    0x00, 0x00, 0x4D, 0xCE, 0x33, 0x24, 0x7C, 0x26, 0x33, 0x02, 0x5C, 0xA7,
    0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x7D, 0x00, 0x32, 0x00, 0x4B,
    0x00, 0xE3, 0xD0, 0x69};
const Bytes kS2_204055000 = {
    0xD3, 0x00, 0x2E, 0xFA, 0x22, 0x09, 0x80, 0x30, 0xA6, 0x87, 0x60, 0x00,
    0x00, 0x00, 0x8D, 0x8E, 0x33, 0x24, 0x6B, 0x11, 0x33, 0x02, 0x5C, 0xCD,
    0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xCB, 0x7D, 0x68};
const Bytes kS3_204000000 = {
    0xD3, 0x00, 0x10, 0xFA, 0x23, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x0D, 0x23, 0xFF, 0xFD, 0xF4, 0x7B, 0x3B, 0x82};
const Bytes kS3_204000160 = {
    0xD3, 0x00, 0x10, 0xFA, 0x23, 0x09, 0x80, 0x30, 0xA3, 0x2E, 0x80, 0x00,
    0x00, 0x00, 0x4D, 0x50, 0x00, 0x01, 0xF4, 0x12, 0x99, 0x02};
const Bytes kS3_204010000 = {
    0xD3, 0x00, 0x10, 0xFA, 0x23, 0x09, 0x80, 0x30, 0xA3, 0xC8, 0x40, 0x00,
    0x00, 0x00, 0x8D, 0x68, 0x59, 0xD9, 0xF4, 0x08, 0x7D, 0x24};
const Bytes kS3_204080100 = {
    0xD3, 0x00, 0x10, 0xFA, 0x23, 0x09, 0x80, 0x30, 0xA8, 0x0F, 0x90, 0x00,
    0x00, 0x00, 0xCD, 0x84, 0x6B, 0xD1, 0xF4, 0xA6, 0xBC, 0x02};
const Bytes kS3_204500000 = {
    0xD3, 0x00, 0x10, 0xFA, 0x23, 0x09, 0x80, 0x30, 0xC1, 0xB0, 0x80, 0x00,
    0x00, 0x01, 0x0D, 0x50, 0x00, 0x01, 0xF4, 0x5F, 0x56, 0x62};
const Bytes kS4_204000000 = {
    0xD3, 0x00, 0x1C, 0xFA, 0x24, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x0D, 0xC6, 0x80, 0x00, 0x0C, 0xFF, 0xDE, 0x00, 0x9C, 0x2D,
    0x41, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x41, 0xDB, 0x0B, 0x57};

// The board's 4001 sub-type 1 epochs. E0: test_rtcm_msm.cpp's epoch as
// today's encoder writes it (no block)
const Bytes kE0 = {
    0xD3, 0x00, 0x1E, 0xFA, 0x11, 0x09, 0x57, 0x90, 0x32, 0x0F, 0xDA, 0x17,
    0xB8, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x29, 0x0C, 0xBA, 0xD6, 0xC4, 0x8D,
    0x15, 0x9E, 0x26, 0xAF, 0x03, 0x0A, 0xEF, 0x01, 0x80, 0x8E, 0xFE, 0x3E};
// E1: E0 with an aiding block
const Bytes kE1 = {
    0xD3, 0x00, 0x22, 0xFA, 0x11, 0x09, 0x57, 0x90, 0x32, 0x0F, 0xDA, 0x17,
    0xB8, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x29, 0x0C, 0xBA, 0xD6, 0xC4, 0x8D,
    0x15, 0x9E, 0x26, 0xAF, 0x03, 0x0A, 0xEF, 0x01, 0x86, 0x34, 0xF0, 0x29,
    0x40, 0x7C, 0x97, 0xF2};
// E2: field ends: 40 marks asked (32 sent) and every version 1 block field at
// its top; version 15 carries version 2's 56 bits more (0): the longest
// epoch, 622 bits
const Bytes kE2 = {
    0xD3, 0x00, 0x4E, 0xFA, 0x11, 0xFF, 0xFF, 0x90, 0x32, 0x0F, 0xFD, 0xFF,
    0xFF, 0xC0, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x05, 0x42, 0xA0, 0xFC, 0x44, 0x16,
    0x46, 0xE1, 0xDC, 0x88, 0x27, 0x4A, 0x62, 0xEC, 0xCC, 0x35, 0x4E, 0xA3,
    0xFD, 0x04, 0x46, 0x52, 0xE4, 0xDD, 0x48, 0x57, 0x56, 0x65, 0xED, 0x8C,
    0x65, 0x5A, 0xA6, 0xFD, 0xC4, 0x76, 0x5E, 0xE7, 0xDE, 0x0B, 0xFF, 0xFF,
    0x83, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF9, 0x17, 0x67};
// E3: no marks, GPS aiding in the boost by the GPS channels' own decision
const Bytes kE3 = {
    0xD3, 0x00, 0x1F, 0xFA, 0x11, 0x09, 0x80, 0x30, 0xA3, 0x2D, 0xE0, 0x01,
    0x34, 0x80, 0x00, 0xC0, 0x00, 0x00, 0x01, 0x24, 0xC4, 0x15, 0xEA, 0xF3,
    0x7B, 0xC0, 0x48, 0xD1, 0x40, 0x00, 0x53, 0x00, 0x00, 0x00, 0xA7, 0xBC,
    0x81};

// E4: E0 with a version 2 block (the board's IMU watch): the IMU refused by
// the pad check in the boost phase, after three drops; the 56 bits after the
// first 32 are the board's (receiver/README.md), not read here
const Bytes kE4 = {
    0xD3, 0x00, 0x29, 0xFA, 0x11, 0x09, 0x57, 0x90, 0x32, 0x0F, 0xDA, 0x17,
    0xB8, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x29, 0x0C, 0xBA, 0xD6, 0xC4, 0x8D,
    0x15, 0x9E, 0x26, 0xAF, 0x03, 0x0A, 0xEF, 0x01, 0x88, 0xA4, 0xB3, 0x8F,
    0x00, 0xD2, 0x40, 0x7F, 0xF7, 0xA0, 0x5C, 0x00, 0x8A, 0xF2, 0xDB};

// ── The inputs the golden frames were made from ─────────────────────────

Header hdr(uint16_t week, uint32_t tow_ms, int32_t sub_ns, uint8_t seq, uint8_t src, bool test, uint8_t tq)
{
    Header h;
    h.week = week;
    h.tow_ms = tow_ms;
    h.sub_ns = sub_ns;
    h.seq = seq;
    h.src = src;
    h.test = test;
    h.tq = tq;
    return h;
}

Attitude att(const Header& h, bool av, bool rv, std::array<double, 4> q, std::array<double, 3> w, double tilt,
             double hdg)
{
    Attitude a;
    a.h = h;
    a.att_valid = av;
    a.rate_valid = rv;
    std::copy(q.begin(), q.end(), a.q);
    std::copy(w.begin(), w.end(), a.w);
    a.sig_tilt_deg = tilt;
    a.sig_hdg_deg = hdg;
    return a;
}

NavState nav(const Header& h, bool pv, bool vv, uint8_t point, std::array<double, 3> pos, std::array<double, 3> vel,
             std::array<double, 4> sig)
{
    NavState n;
    n.h = h;
    n.pos_valid = pv;
    n.vel_valid = vv;
    n.point = point;
    std::copy(pos.begin(), pos.end(), n.pos);
    std::copy(vel.begin(), vel.end(), n.vel);
    std::copy(sig.begin(), sig.end(), n.sig);
    return n;
}

PhaseMsg phase(const Header& h, uint8_t ph, uint8_t loops, bool canopy, uint32_t since_ms, uint32_t hold_ms)
{
    PhaseMsg p;
    p.h = h;
    p.phase = ph;
    p.loops = loops;
    p.canopy = canopy;
    p.since_ms = since_ms;
    p.hold_ms = hold_ms;
    return p;
}

Rest rest(const Header& h, bool valid, uint16_t dur_ms, std::array<double, 3> f, double sd_f)
{
    Rest r;
    r.h = h;
    r.valid = valid;
    r.dur_ms = dur_ms;
    std::copy(f.begin(), f.end(), r.f);
    r.sd_f = sd_f;
    return r;
}

Geometry geom(const Header& h, bool lv, bool rv, uint8_t type, uint8_t axis, std::array<double, 3> lever,
              std::array<double, 4> rot)
{
    Geometry g;
    g.h = h;
    g.lever_valid = lv;
    g.rot_valid = rv;
    g.ant_type = type;
    g.ant_axis = axis;
    std::copy(lever.begin(), lever.end(), g.lever_m);
    std::copy(rot.begin(), rot.end(), g.rot);
    return g;
}

const std::array<double, 3> kN1Pos = {1288245.1234, -4720736.5678, 4079612.9012};
const std::array<double, 4> kN1Sig = {2.5, 4.0, 0.15, 0.25};

const Attitude kInA1 = att(hdr(2432, 204000123, 456789, 7, kSrcFusedBoard, false, 3), true, true,
                           {0.8, -0.36, 0.48, 0.0}, {0.5, -1.25, 3.0}, 1.25, 7.3);
const Attitude kInA2 = att(hdr(65535, 604799999, -600000, 255, kSrcFusedOther, true, 7), true, true,
                           {-0.6, 0.0, 0.0, -0.8}, {200.0, -0.00048828125, -127.9990234375}, 30.0, kNaN);
const Attitude kInA3 = att(hdr(2432, 1, 0, 0, kSrcOwn, false, 0), false, false, {0.1, 0.2, 0.3, 0.4},
                           {1.0, 2.0, 3.0}, 1.0, 1.0);
const Attitude kInA4 = att(hdr(2432, 204000200, 0, 9, kSrcOwn, false, 3), true, true, {kNaN, 0.0, 0.0, 1.0},
                           {kInf, -kInf, 0.5}, 1.0, 2.0);
const NavState kInN1 = nav(hdr(2432, 204000050, 0, 0, kSrcFusedBoard, false, 3), true, true, 0, kN1Pos,
                           {-0.1234, 812.3456, -4.0001}, kN1Sig);
const NavState kInN2 = nav(hdr(0, 0, 524287, 128, kSrcOwn, false, 1), true, true, 1, {1.4e7, -1.4e7, -0.00005},
                           {1.4e4, -1.4e4, 0.00005}, {700.0, 0.0, kNaN, -1.0});
const NavState kInN3 = nav(hdr(2432, 204055000, -1, 1, kSrcScript, true, 4), true, false, 0,
                           {-3092763.9484, -5579493.8587, 0.0}, {9.0, 9.0, 9.0}, {3.0, 5.0, 0.2, 0.3});
const NavState kInN4 = nav(hdr(2432, 204000250, 0, 10, kSrcOwn, false, 3), true, true, 0, kN1Pos, {0.0, kNaN, 0.0},
                           kN1Sig);
const PhaseMsg kInP1 = phase(hdr(2432, 204001100, 0, 3, kSrcScript, true, 4), kPhBoost, kLoopsBoost, false, 1040, 2500);
const PhaseMsg kInP2 = phase(hdr(0, 0, 524287, 255, kSrcOwn, false, 7), kPhLanded, kLoopsQuiet, true, 70000, 9000);
const PhaseMsg kInP3 = phase(hdr(2432, 204010150, -400000, 7, kSrcFusedBoard, false, 3), kPhCoast, kLoopsNone, false,
                             5900, 0);
const Geometry kInG1 = geom(hdr(2432, 204000000, 0, 1, kSrcScript, true, 4), true, true, 1, 5, {0.012, -0.034, 0.156},
                            {kR2, 0.0, 0.0, kR2});
const Geometry kInG2 = geom(hdr(65535, 604799999, -524287, 255, kSrcFusedOther, false, 7), true, true, 15, 7,
                            {40.0, -40.0, -0.0005}, {-0.5, 0.5, -0.5, 0.5});
const Geometry kInG3 = geom(hdr(2432, 204000000, 0, 2, kSrcOwn, false, 0), false, false, 0, 0, {1.0, 1.0, 1.0},
                            {1.0, 0.0, 0.0, 0.0});
const Geometry kInG4 = geom(hdr(2432, 204000300, 0, 11, kSrcOwn, false, 0), true, true, 1, 5, {kNaN, 0.0, 0.0},
                            {kR2, 0.0, 0.0, kR2});
const Rest kInRest1 = rest(hdr(2432, 204000000, 0, 1, kSrcOwn, false, 5), true, 1000, {0.0123, -0.4567, -9.7801}, 0.0042);
const Rest kInRest2 = rest(hdr(65535, 604799999, -524287, 255, kSrcFusedOther, true, 7), true, 5000,
                        {60.0, -60.0, -0.00005}, kNaN);
const Rest kInRest3 = rest(hdr(2432, 204000500, 0, 2, kSrcOwn, false, 5), false, 1000, {1.0, 2.0, 3.0}, 0.0004);
const Rest kInRest4 = rest(hdr(2432, 204005000, 0, 3, kSrcOwn, false, 5), true, 1000, {0.0, kNaN, -9.8}, 0.003);

// ── Helpers ─────────────────────────────────────────────────────────────

template <typename T>
Bytes enc(const T& v)
{
    uint8_t b[kMaxFrame];
    const size_t n = encode(v, b, sizeof(b));
    return Bytes(b, b + n);
}

// An RTCM3 frame around `payload`, its CRC-24Q computed.
Bytes frameOf(const Bytes& payload)
{
    Bytes f = {0xD3, static_cast<uint8_t>(payload.size() >> 8), static_cast<uint8_t>(payload.size())};
    f.insert(f.end(), payload.begin(), payload.end());
    const uint32_t crc = rtcm3::crc24q(f.data(), f.size());
    f.push_back(static_cast<uint8_t>(crc >> 16));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    f.push_back(static_cast<uint8_t>(crc));
    return f;
}

Bytes payloadOf(const Bytes& frame) { return Bytes(frame.begin() + 3, frame.end() - 3); }

// A decoded value in its field's LSBs, and a 1-sigma's code (NaN: all ones).
int64_t lsb(double v, double scale) { return std::llround(v * scale); }
int64_t code(double sig, double scale, int64_t top) { return std::isnan(sig) ? top : std::llround(sig * scale); }

void expectHeader(const Header& got, const Header& sent)
{
    EXPECT_EQ(got.week, sent.week);
    EXPECT_EQ(got.tow_ms, sent.tow_ms);
    EXPECT_EQ(got.sub_ns, std::max(-524287, std::min(524287, sent.sub_ns)));   // clamped to its 20 bits
    EXPECT_EQ(got.seq, sent.seq);
    EXPECT_EQ(got.src, sent.src);
    EXPECT_EQ(got.test, sent.test);
    EXPECT_EQ(got.tq, sent.tq);
}

void checkAtt(const char* name, const Bytes& want, const Attitude& in, bool av, bool rv, std::array<int64_t, 4> q,
              std::array<int64_t, 3> w, int64_t tilt, int64_t hdg)
{
    SCOPED_TRACE(name);
    EXPECT_EQ(enc(in), want);
    Attitude d;
    ASSERT_TRUE(decode(want.data(), want.size(), d));
    expectHeader(d.h, in.h);
    EXPECT_EQ(d.att_valid, av);
    EXPECT_EQ(d.rate_valid, rv);
    EXPECT_EQ(d.frame, 0);
    for (int k = 0; k < 4; k++) EXPECT_EQ(lsb(d.q[k], 16384.0), q[k]) << "q" << k;
    for (int k = 0; k < 3; k++) EXPECT_EQ(lsb(d.w[k], 1024.0), w[k]) << "w" << k;
    EXPECT_EQ(code(d.sig_tilt_deg, 10.0, 255), tilt);
    EXPECT_EQ(code(d.sig_hdg_deg, 2.0, 255), hdg);
    EXPECT_EQ(std::isnan(d.sig_tilt_deg), tilt == 255);   // all ones reads unknown, not 25.5 deg
    EXPECT_EQ(std::isnan(d.sig_hdg_deg), hdg == 255);
    EXPECT_EQ(enc(d), want);   // what a decoder reads goes out as the same frame
}

void checkNav(const char* name, const Bytes& want, const NavState& in, bool pv, bool vv, std::array<int64_t, 3> pos,
              std::array<int64_t, 3> vel, std::array<int64_t, 4> sig)
{
    SCOPED_TRACE(name);
    EXPECT_EQ(enc(in), want);
    NavState d;
    ASSERT_TRUE(decode(want.data(), want.size(), d));
    expectHeader(d.h, in.h);
    EXPECT_EQ(d.pos_valid, pv);
    EXPECT_EQ(d.vel_valid, vv);
    EXPECT_EQ(d.point, in.point);
    for (int k = 0; k < 3; k++) EXPECT_EQ(lsb(d.pos[k], 1e4), pos[k]) << "pos" << k;
    for (int k = 0; k < 3; k++) EXPECT_EQ(lsb(d.vel[k], 1e4), vel[k]) << "vel" << k;
    EXPECT_EQ(code(d.sig[0], 100.0, 65535), sig[0]);
    EXPECT_EQ(code(d.sig[1], 100.0, 65535), sig[1]);
    EXPECT_EQ(code(d.sig[2], 1000.0, 65535), sig[2]);
    EXPECT_EQ(code(d.sig[3], 1000.0, 65535), sig[3]);
    for (int k = 0; k < 4; k++) EXPECT_EQ(std::isnan(d.sig[k]), sig[k] == 65535) << "sigma " << k;
    EXPECT_EQ(enc(d), want);
}

void checkPhase(const char* name, const Bytes& want, const PhaseMsg& in, uint32_t since_ms, uint32_t hold)
{
    SCOPED_TRACE(name);
    EXPECT_EQ(enc(in), want);
    PhaseMsg d;
    ASSERT_TRUE(decode(want.data(), want.size(), d));
    expectHeader(d.h, in.h);
    EXPECT_EQ(d.phase, in.phase);
    EXPECT_EQ(d.loops, in.loops);
    EXPECT_EQ(d.canopy, in.canopy);
    EXPECT_EQ(d.since_ms, since_ms);
    EXPECT_EQ(d.hold_ms, hold * 20);
    EXPECT_EQ(enc(d), want);
}

void checkGeom(const char* name, const Bytes& want, const Geometry& in, bool lv, bool rv, std::array<int64_t, 3> mm,
               std::array<int64_t, 4> rot)
{
    SCOPED_TRACE(name);
    EXPECT_EQ(enc(in), want);
    Geometry d;
    ASSERT_TRUE(decode(want.data(), want.size(), d));
    expectHeader(d.h, in.h);
    EXPECT_EQ(d.lever_valid, lv);
    EXPECT_EQ(d.rot_valid, rv);
    EXPECT_EQ(d.ant_type, in.ant_type);
    EXPECT_EQ(d.ant_axis, in.ant_axis);
    for (int k = 0; k < 3; k++) EXPECT_EQ(lsb(d.lever_m[k], 1000.0), mm[k]) << "lever" << k;
    for (int k = 0; k < 4; k++) EXPECT_EQ(lsb(d.rot[k], 16384.0), rot[k]) << "r" << k;
    EXPECT_EQ(enc(d), want);
}

void checkRest(const char* name, const Bytes& want, const Rest& in, bool valid, uint32_t dur_ms,
               std::array<int64_t, 3> f, int64_t sd)
{
    SCOPED_TRACE(name);
    EXPECT_EQ(enc(in), want);
    Rest d;
    ASSERT_TRUE(decode(want.data(), want.size(), d));
    expectHeader(d.h, in.h);
    EXPECT_EQ(d.valid, valid);
    EXPECT_EQ(d.dur_ms, dur_ms);
    for (int k = 0; k < 3; k++) EXPECT_EQ(lsb(d.f[k], 10000.0), f[k]) << "f" << k;
    EXPECT_EQ(code(d.sd_f, 1000.0, 255), sd);
    EXPECT_EQ(std::isnan(d.sd_f), sd == 255);   // all ones reads unknown, not 0.255 m/s^2
    EXPECT_EQ(enc(d), want);
}

// Every golden 4002 frame, its sub-type with it.
std::vector<std::pair<uint8_t, Bytes>> allFc()
{
    return {{kAttitude, kA1}, {kAttitude, kA2}, {kAttitude, kA3}, {kAttitude, kA4}, {kNavState, kN1},
            {kNavState, kN2}, {kNavState, kN3}, {kNavState, kN4}, {kPhase, kP1}, {kPhase, kP2},
            {kPhase, kP3}, {kGeometry, kG1}, {kGeometry, kG2}, {kGeometry, kG3}, {kGeometry, kG4},
            {kAttitude, kS1_204000000}, {kAttitude, kS1_204005000}, {kAttitude, kS1_204017250},
            {kNavState, kS2_204000000}, {kNavState, kS2_204002500}, {kNavState, kS2_204055000},
            {kPhase, kS3_204000000}, {kPhase, kS3_204000160}, {kPhase, kS3_204010000},
            {kPhase, kS3_204080100}, {kPhase, kS3_204500000}, {kGeometry, kS4_204000000},
            {kRest, kRest1}, {kRest, kRest2}, {kRest, kRest3}, {kRest, kRest4},
            {kAttitude, kA5}, {kRest, kRest5}};
}

// What a sub-type's decoder reads of a frame, encoded again (nothing if it
// refuses the frame).
template <typename T>
Bytes reread(const Bytes& f)
{
    T v;
    return decode(f.data(), f.size(), v) ? enc(v) : Bytes{};
}

Bytes rereadAs(uint8_t sub, const Bytes& f)
{
    switch (sub)
    {
        case kAttitude: return reread<Attitude>(f);
        case kNavState: return reread<NavState>(f);
        case kPhase: return reread<PhaseMsg>(f);
        case kRest: return reread<Rest>(f);
        default: return reread<Geometry>(f);
    }
}

// Whether each of the five decoders takes a frame.
using Taken = std::array<bool, 5>;
constexpr Taken kNone = {false, false, false, false, false};
Taken takenBy(const Bytes& f)
{
    Attitude a;
    NavState n;
    PhaseMsg p;
    Geometry g;
    Rest r;
    return {decode(f.data(), f.size(), a), decode(f.data(), f.size(), n), decode(f.data(), f.size(), p),
            decode(f.data(), f.size(), g), decode(f.data(), f.size(), r)};
}

}  // namespace

// ── The golden frames ───────────────────────────────────────────────────

TEST(SdrAid, TheGoldenFrames)
{
    checkAtt("A1", kA1, kInA1, true, true, {13107, -5898, 7864, 0}, {512, -1280, 3072}, 13, 15);
    checkNav("N1", kN1, kInN1, true, true, {12882451234, -47207365678, 40796129012}, {-1234, 8123456, -40001},
             {250, 400, 150, 250});
    checkPhase("P1", kP1, kInP1, 1040, 125);
    checkPhase("P3", kP3, kInP3, 5900, 0);
    checkGeom("G1", kG1, kInG1, true, true, {12, -34, 156}, {11585, 0, 0, 11585});
    checkRest("R1", kRest1, kInRest1, true, 1000, {123, -4567, -97801}, 5);
    // Nothing known: the fields go out as 0 whatever the struct held (a
    // rest's sigma goes as it is)
    checkAtt("A3", kA3, kInA3, false, false, {0, 0, 0, 0}, {0, 0, 0}, 0, 0);
    checkNav("N3", kN3, kInN3, true, false, {-30927639484, -55794938587, 0}, {0, 0, 0}, {300, 500, 0, 0});
    checkGeom("G3", kG3, kInG3, false, false, {0, 0, 0}, {0, 0, 0, 0});
    checkRest("R3", kRest3, kInRest3, false, 1000, {0, 0, 0}, 1);
}

TEST(SdrAid, TheImuFrameGoldens)
{
    // The same values in the flight computer's IMU axes: the frame field (bits
    // 98-99 of the attitude, bit 97 of the rest) and nothing else differs
    Attitude a = kInA1;
    a.frame = kFrameImu;
    EXPECT_EQ(enc(a), kA5);
    Rest r = kInRest1;
    r.frame = kFrameImu;
    EXPECT_EQ(enc(r), kRest5);
    Attitude da;
    ASSERT_TRUE(decode(kA5.data(), kA5.size(), da));
    EXPECT_EQ(da.frame, kFrameImu);
    EXPECT_EQ(enc(da), kA5);
    Rest dr;
    ASSERT_TRUE(decode(kRest5.data(), kRest5.size(), dr));
    EXPECT_EQ(dr.frame, kFrameImu);
    EXPECT_TRUE(dr.valid);
    EXPECT_EQ(dr.dur_ms, 1000);
    EXPECT_EQ(enc(dr), kRest5);
    // and the body's goldens read frame 0
    ASSERT_TRUE(decode(kRest1.data(), kRest1.size(), dr));
    EXPECT_EQ(dr.frame, kFrameBody);
    for (size_t i = 0; i < kA5.size() - 3; i++)
        if (i != 15) EXPECT_EQ(kA5[i], kA1[i]) << i;   // byte 15 holds bits 96-103 of the message
    for (size_t i = 0; i < kRest5.size() - 3; i++)
        if (i != 15) EXPECT_EQ(kRest5[i], kRest1[i]) << i;
}

TEST(SdrAid, TheFieldEnds)
{
    // Saturated past the field, a half LSB away from zero, a negative scalar
    // part sent negated, sigmas past the field or unknown all ones, a zero
    // sigma 1, the header at its tops.
    checkAtt("A2", kA2, kInA2, true, true, {9830, 0, 0, 13107}, {131071, -1, -131071}, 255, 255);
    checkNav("N2", kN2, kInN2, true, true, {137438953471, -137438953471, -1}, {134217727, -134217727, 1},
             {65535, 1, 65535, 65535});
    checkPhase("P2", kP2, kInP2, 65535, 255);
    checkGeom("G2", kG2, kInG2, true, true, {32767, -32767, -1}, {8192, -8192, 8192, -8192});
    checkRest("R2", kRest2, kInRest2, true, 4095, {524287, -524287, -1}, 255);

    // A sigma within a thousandth of an LSB over it is that LSB (0.07 m is
    // 7.000000000000001 cm in doubles: 7, not 8), one further over is the
    // next, and the hold goes to the nearest 20 ms (the reference codec's
    // codes)
    NavState n = kInN1;
    n.sig[0] = 0.07;
    n.sig[1] = 0.07005;
    n.sig[2] = 0.0070000001;
    n.sig[3] = 0.0071;
    NavState dn;
    Bytes f = enc(n);
    ASSERT_TRUE(decode(f.data(), f.size(), dn));
    EXPECT_EQ(code(dn.sig[0], 100.0, 65535), 7);
    EXPECT_EQ(code(dn.sig[1], 100.0, 65535), 8);
    EXPECT_EQ(code(dn.sig[2], 1000.0, 65535), 7);
    EXPECT_EQ(code(dn.sig[3], 1000.0, 65535), 8);
    for (const auto& [hold_ms, steps] : {std::pair<uint32_t, uint32_t>{2509, 125}, {2510, 126}, {9, 0}, {10, 1}})
    {
        PhaseMsg p = kInP1;
        p.hold_ms = hold_ms;
        PhaseMsg dp;
        f = enc(p);
        ASSERT_TRUE(decode(f.data(), f.size(), dp));
        EXPECT_EQ(dp.hold_ms, steps * 20) << hold_ms;
    }
}

TEST(SdrAid, ANaNSendsItsGroupAsNotValid)
{
    // A filter gone bad: a NaN clears its group (and the sigmas that go with
    // it), an infinity saturates, the other groups go out as they are.
    checkAtt("A4", kA4, kInA4, false, true, {0, 0, 0, 0}, {131071, -131071, 512}, 0, 0);
    checkNav("N4", kN4, kInN4, true, false, {12882451234, -47207365678, 40796129012}, {0, 0, 0}, {250, 400, 0, 0});
    checkGeom("G4", kG4, kInG4, false, true, {0, 0, 0}, {11585, 0, 0, 11585});
    checkRest("R4", kRest4, kInRest4, false, 1000, {0, 0, 0}, 3);
    // A NaN in the rates and in the position the same way, the quaternion and
    // the velocity kept (the reference codec's bytes for A1 and N1 so)
    Attitude a = kInA1;
    a.w[1] = kNaN;
    checkAtt("A1, a NaN rate", {0xD3, 0x00, 0x1E, 0xFA, 0x21, 0x09, 0x80, 0x30, 0xA3, 0x2D, 0xED, 0xBE,
                                0x15, 0x41, 0xE3, 0x83, 0x33, 0x3E, 0x8F, 0x61, 0xEB, 0x80, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x43, 0xC0, 0x8C, 0x11, 0x37},
             a, true, false, {13107, -5898, 7864, 0}, {0, 0, 0}, 13, 15);
    NavState n = kInN1;
    n.pos[0] = kNaN;
    checkNav("N1, a NaN position", {0xD3, 0x00, 0x2E, 0xFA, 0x22, 0x09, 0x80, 0x30, 0xA3, 0x2C, 0xC8, 0x00,
                                    0x00, 0x00, 0x23, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0xCB, 0x81, 0xEF, 0xD1,
                                    0x03, 0xFF, 0xD8, 0xEF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x25, 0x80, 0x3E,
                                    0x80, 0x10, 0x32, 0x9C},
             n, false, true, {0, 0, 0}, {-1234, 8123456, -40001}, {0, 0, 150, 250});
    // A NaN rotation the same way, the lever arm kept
    Geometry g = kInG1;
    g.rot[2] = kNaN;
    Geometry d;
    const Bytes f = enc(g);
    ASSERT_TRUE(decode(f.data(), f.size(), d));
    EXPECT_TRUE(d.lever_valid);
    EXPECT_FALSE(d.rot_valid);
    EXPECT_EQ(lsb(d.lever_m[2], 1000.0), 156);
    EXPECT_EQ(lsb(d.rot[0], 16384.0), 0);
}

TEST(SdrAid, AQuaternionAndItsNegativeMakeOneFrame)
{
    Attitude a = kInA1;
    for (double& x : a.q) x = -x;
    EXPECT_EQ(enc(a), kA1);
    Geometry g = kInG1;
    for (double& x : g.rot) x = -x;
    EXPECT_EQ(enc(g), kG1);
}

TEST(SdrAid, AShortBufferGetsNothing)
{
    uint8_t b[kMaxFrame];
    EXPECT_EQ(encode(kInA1, b, kAttitudeFrame - 1), 0u);
    EXPECT_EQ(encode(kInN1, b, kNavStateFrame - 1), 0u);
    EXPECT_EQ(encode(kInP1, b, kPhaseFrame - 1), 0u);
    EXPECT_EQ(encode(kInG1, b, kGeometryFrame - 1), 0u);
    EXPECT_EQ(encode(kInRest1, b, kRestFrame - 1), 0u);
    EXPECT_EQ(encode(kInA1, nullptr, sizeof(b)), 0u);
    EXPECT_EQ(encode(kInA1, b, kAttitudeFrame), kAttitudeFrame);
    EXPECT_EQ(encode(kInN1, b, kNavStateFrame), kNavStateFrame);
    EXPECT_EQ(encode(kInP1, b, kPhaseFrame), kPhaseFrame);
    EXPECT_EQ(encode(kInG1, b, kGeometryFrame), kGeometryFrame);
    EXPECT_EQ(encode(kInRest1, b, kRestFrame), kRestFrame);
    EXPECT_EQ(kRestFrame, 29u);
    EXPECT_LE(kRestFrame, kMaxFrame);
}

TEST(SdrAid, ShortFramesRefusedLongerOnesReadTheSame)
{
    for (const auto& [sub, f] : allFc())
    {
        // One byte short, re-framed: refused by every decoder
        Bytes p = payloadOf(f);
        p.pop_back();
        EXPECT_EQ(takenBy(frameOf(p)), kNone);
        // Four bytes more (a later revision's), re-framed: read as before
        p = payloadOf(f);
        p.insert(p.end(), {0xA5, 0x5A, 0xFF, 0x00});
        const Bytes longer = frameOf(p);
        Taken want = kNone;
        want[sub - 1] = true;
        EXPECT_EQ(takenBy(longer), want);
        EXPECT_EQ(rereadAs(sub, longer), f);
    }
}

TEST(SdrAid, EachDecoderTakesOnlyItsOwn)
{
    for (const auto& [sub, f] : allFc())
    {
        Taken want = kNone;
        want[sub - 1] = true;
        EXPECT_EQ(takenBy(f), want);
        EXPECT_EQ(fcSubType(f.data(), f.size()), sub);
        EXPECT_EQ(sdr_rtcm::messageNumber(f.data(), f.size()), kFcMessage);
        EXPECT_EQ(sdr_rtcm::trSubType(f.data(), f.size()), 0);   // no 4001
        sdr_rtcm::TrEpoch e;
        EpochAid aid;
        EXPECT_FALSE(sdr_rtcm::decodeTrEpoch(f.data(), f.size(), e));
        EXPECT_FALSE(decodeEpochAid(f.data(), f.size(), aid));
        // The same bytes as a reserved sub-type (0, 6-15): nobody's
        for (uint8_t other : {0, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15})
        {
            Bytes p = payloadOf(f);
            p[1] = static_cast<uint8_t>((p[1] & 0xF0) | other);
            const Bytes g = frameOf(p);
            EXPECT_EQ(fcSubType(g.data(), g.size()), other);
            EXPECT_EQ(takenBy(g), kNone) << int(sub) << " as " << int(other);
        }
        // As a rest: the rest's decoder alone, and only one at least a
        // rest's 23 bytes long (not a phase's 16)
        if (sub != kRest)
        {
            Bytes p = payloadOf(f);
            p[1] = static_cast<uint8_t>((p[1] & 0xF0) | kRest);
            const Bytes g = frameOf(p);
            Taken as_rest = kNone;
            as_rest[kRest - 1] = p.size() >= kRestBytes;
            EXPECT_EQ(takenBy(g), as_rest) << int(sub) << " as a rest";
        }
    }
    // The board's 4001 epochs are no 4002
    for (const Bytes* e : {&kE0, &kE1, &kE2, &kE3, &kE4})
    {
        EXPECT_EQ(fcSubType(e->data(), e->size()), 0);
        EXPECT_EQ(takenBy(*e), kNone);
    }
    EXPECT_EQ(fcSubType(nullptr, 0), 0);
}

// ── The stamp ───────────────────────────────────────────────────────────

TEST(SdrAid, TheStamp)
{
    // The nearest ms, the rest in ns; the week's end carried into the next
    struct Row { int week_in; double tow_s; uint16_t week; uint32_t tow_ms; int32_t sub_ns; };
    const Row rows[] = {
        {2432, 204000.1234567891, 2432, 204000123, 456789},
        {2432, 204000.0004996, 2432, 204000000, 499600},
        {2432, 204000.0005004, 2432, 204000001, -499600},
        {2432, 604799.9996, 2433, 0, -400000},
        {2432, 604799.99949, 2432, 604799999, 490000},
        {2432, 0.0, 2432, 0, 0},
    };
    for (const Row& r : rows)
    {
        const Header h = stampAt(r.week_in, r.tow_s);
        EXPECT_EQ(h.week, r.week) << r.tow_s;
        EXPECT_EQ(h.tow_ms, r.tow_ms) << r.tow_s;
        EXPECT_EQ(h.sub_ns, r.sub_ns) << r.tow_s;
    }
}

TEST(SdrAid, AScriptFramesStampIsItsWholeMs)
{
    // The bench's header from the GPS time its line gives: week and time of
    // week split off, the nearest ms, sub_ns 0, a script test frame at tq 5.
    const Header h = scriptStamp(2432 * 604800.0 + 204017.2496);
    EXPECT_EQ(h.week, 2432);
    EXPECT_EQ(h.tow_ms, 204017250u);
    EXPECT_EQ(h.sub_ns, 0);
    EXPECT_EQ(h.src, kSrcScript);
    EXPECT_TRUE(h.test);
    EXPECT_EQ(h.tq, kScriptTq);
    EXPECT_EQ(h.tq, 5);
    // The week's last half ms is the next week's 0, and the next week's first
    // ms is its own
    const Header e = scriptStamp(2433 * 604800.0 - 0.0004);
    EXPECT_EQ(e.week, 2433);
    EXPECT_EQ(e.tow_ms, 0u);
    const Header n = scriptStamp(2433 * 604800.0 + 0.0012);
    EXPECT_EQ(n.week, 2433);
    EXPECT_EQ(n.tow_ms, 1u);
}

// ── The script ──────────────────────────────────────────────────────────

TEST(SdrAid, TheScript)
{
    auto at = [](uint32_t tow_ms, uint8_t seq) { return hdr(2432, tow_ms, 0, seq, kSrcScript, true, kScriptTq); };
    EXPECT_EQ(enc(scriptAttitude(at(204000000, 0))), kS1_204000000);   // roll -60 deg
    EXPECT_EQ(enc(scriptAttitude(at(204005000, 1))), kS1_204005000);
    EXPECT_EQ(enc(scriptAttitude(at(204017250, 2))), kS1_204017250);
    EXPECT_EQ(enc(scriptNav(at(204000000, 0))), kS2_204000000);
    EXPECT_EQ(enc(scriptNav(at(204002500, 1))), kS2_204002500);
    EXPECT_EQ(enc(scriptNav(at(204055000, 2))), kS2_204055000);        // the velocity's invalid 10 s
    EXPECT_EQ(enc(scriptPhase(at(204000000, 0))), kS3_204000000);      // the pad
    EXPECT_EQ(enc(scriptPhase(at(204000160, 1))), kS3_204000160);      // the boost, its first ms
    EXPECT_EQ(enc(scriptPhase(at(204010000, 2))), kS3_204010000);      // the coast
    EXPECT_EQ(enc(scriptPhase(at(204080100, 3))), kS3_204080100);      // descent under the canopy
    EXPECT_EQ(enc(scriptPhase(at(204500000, 4))), kS3_204500000);      // the cycle's boost
    EXPECT_EQ(enc(scriptGeometry(at(204000000, 0))), kS4_204000000);

    // A unit quaternion whose scalar part stays clear of the sign flip, and
    // the roll rate it turns at: one roll a minute
    for (uint32_t t = 0; t < 20000; t += 7)
    {
        const Attitude a = scriptAttitude(at(204000000 + t, 0));
        EXPECT_NEAR(a.q[0] * a.q[0] + a.q[1] * a.q[1] + a.q[2] * a.q[2] + a.q[3] * a.q[3], 1.0, 1e-12);
        EXPECT_GT(a.q[0], 0.3);
        EXPECT_DOUBLE_EQ(a.w[0], 2.0 * 3.141592653589793 / 60.0);
    }

    // The velocity's invalid 10 s of each minute, at its edges
    for (const auto& [tow_ms, valid] : {std::pair<uint32_t, bool>{204049999, true}, {204050000, false},
                                        {204059999, false}, {204060000, true}})
    {
        const NavState n = scriptNav(at(tow_ms, 0));
        EXPECT_TRUE(n.pos_valid) << tow_ms;
        EXPECT_EQ(n.vel_valid, valid) << tow_ms;
    }
}

TEST(SdrAid, TheScriptsPhases)
{
    // The scene's edges, the cycle's, and the week's ends (the P1 spec's
    // phase_rows: the board checks the same), then each of the cycle's edges
    // from both sides (the reference codec's values)
    struct Row { uint32_t tow_ms; uint8_t phase, loops; bool canopy; uint32_t since_ms; };
    const Row rows[] = {
        {203880100, 1, 0, 0, 65535}, {203880099, 1, 0, 0, 65535}, {204000159, 1, 0, 0, 65535},
        {204000160, 2, 2, 0, 0},     {204004249, 2, 2, 0, 4089},  {204004250, 3, 1, 0, 0},
        {204073199, 3, 1, 0, 65535}, {204073200, 4, 0, 0, 0},     {204080099, 4, 0, 0, 6899},
        {204080100, 4, 0, 1, 6900},  {204300099, 4, 0, 1, 65535}, {204300100, 4, 0, 0, 100},
        {204320000, 4, 0, 1, 20000}, {204339999, 4, 0, 1, 39999}, {204340000, 4, 0, 1, 40000},
        {204344000, 4, 0, 1, 44000}, {204380000, 2, 2, 0, 0},     {204390000, 3, 1, 0, 6000},
        {204430000, 4, 0, 1, 10000}, {1, 1, 0, 0, 65535},         {604799999, 5, 0, 0, 9999},
        {204379999, 1, 0, 0, 65535}, {204383999, 2, 2, 0, 3999},  {204384000, 3, 1, 0, 0},
        {204419999, 3, 1, 0, 35999}, {204420000, 4, 0, 0, 0},     {204429999, 4, 0, 0, 9999},
        {204469999, 4, 0, 1, 49999}, {204470000, 5, 0, 0, 0},     {204479999, 5, 0, 0, 9999},
        {204480000, 1, 0, 0, 65535},
    };
    for (const Row& r : rows)
    {
        const PhaseMsg p = scriptPhase(hdr(2432, r.tow_ms, 0, 0, kSrcScript, true, kScriptTq));
        EXPECT_EQ(p.phase, r.phase) << r.tow_ms;
        EXPECT_EQ(p.loops, r.loops) << r.tow_ms;
        EXPECT_EQ(p.canopy, r.canopy) << r.tow_ms;
        EXPECT_EQ(p.since_ms, r.since_ms) << r.tow_ms;
        EXPECT_EQ(p.hold_ms, kScriptHoldMs);
        EXPECT_EQ(scriptPhaseKey(r.tow_ms), r.phase | r.loops << 3 | (r.canopy ? 1u : 0u) << 5) << r.tow_ms;
    }
}

// ── The board's 4001 epochs ─────────────────────────────────────────────

TEST(SdrAid, TheBoardsEpochs)
{
    // SdrRtcm's decoder (unchanged) reads an epoch with the block as without
    // it; decodeEpochAid finds the block and each epoch's shift, and the
    // latch is the tag less the shift (GPS seconds: E0's 0.5 ms after its
    // tag), but for E2, which the board marks late (NaN: left off the line)
    struct Want { const Bytes* f; uint16_t week; uint32_t tow_ms; uint8_t mode, nobs; int32_t shift_ns; double latch; };
    const Want rows[] = {
        {&kE0, 2391, 604799990, 1, 27, -499999, 1446681599.990499999},
        {&kE1, 2391, 604799990, 1, 27, -499999, 1446681599.990499999},
        {&kE2, 65535, 604799999, 3, 63, 524287, kNaN},
        {&kE3, 2432, 204000120, 1, 23, 1234, 1471077600.119998766},
        {&kE4, 2391, 604799990, 1, 27, -499999, 1446681599.990499999},
    };
    for (const Want& w : rows)
    {
        sdr_rtcm::TrEpoch e;
        ASSERT_TRUE(sdr_rtcm::decodeTrEpoch(w.f->data(), w.f->size(), e));
        EXPECT_EQ(e.week, w.week);
        EXPECT_EQ(e.tow_ms, w.tow_ms);
        EXPECT_EQ(e.mode, w.mode);
        EXPECT_EQ(e.nobs, w.nobs);
        EpochAid a;
        ASSERT_TRUE(decodeEpochAid(w.f->data(), w.f->size(), a));
        EXPECT_EQ(a.shift_ns, w.shift_ns);
        double latch = 0.0;
        ASSERT_EQ(latchGps(e, a, latch), !std::isnan(w.latch)) << w.tow_ms;
        if (!std::isnan(w.latch))
        {
            EXPECT_NEAR(latch, w.latch, 1e-5) << w.tow_ms;
        }
    }

    EpochAid a;
    ASSERT_TRUE(decodeEpochAid(kE0.data(), kE0.size(), a));
    EXPECT_FALSE(a.present);   // today's encoder: none
    EXPECT_EQ(a.ver, 0);
    EXPECT_EQ(a.fc_frames, 0);

    ASSERT_TRUE(decodeEpochAid(kE1.data(), kE1.size(), a));
    EXPECT_TRUE(a.present);
    EXPECT_EQ(a.ver, 1);
    EXPECT_EQ(a.src, 2);       // the board's IMU
    EXPECT_EQ(a.check, 0);
    EXPECT_TRUE(a.boost);
    EXPECT_TRUE(a.wide);
    EXPECT_EQ(a.by, 1);        // the flight computer's phase
    EXPECT_FALSE(a.planned);
    EXPECT_EQ(a.phase, kPhCoast);
    EXPECT_TRUE(a.att);
    EXPECT_TRUE(a.nav);
    EXPECT_FALSE(a.late);
    EXPECT_EQ(a.fc_frames, 0xA5);

    ASSERT_TRUE(decodeEpochAid(kE2.data(), kE2.size(), a));   // after 32 marks, every field at its top
    EXPECT_TRUE(a.present);
    EXPECT_EQ(a.ver, 15);
    EXPECT_EQ(a.src, 3);
    EXPECT_EQ(a.check, 3);
    EXPECT_TRUE(a.boost);
    EXPECT_TRUE(a.wide);
    EXPECT_EQ(a.by, 3);
    EXPECT_TRUE(a.planned);
    EXPECT_EQ(a.phase, 7);
    EXPECT_TRUE(a.att);
    EXPECT_TRUE(a.nav);
    EXPECT_TRUE(a.late);
    EXPECT_EQ(a.fc_frames, 0xFF);

    ASSERT_TRUE(decodeEpochAid(kE3.data(), kE3.size(), a));
    EXPECT_TRUE(a.present);
    EXPECT_EQ(a.ver, 1);
    EXPECT_EQ(a.src, 1);       // GPS-derived
    EXPECT_TRUE(a.boost);
    EXPECT_TRUE(a.wide);
    EXPECT_EQ(a.by, 0);        // the GPS channels
    EXPECT_EQ(a.phase, 0);
    EXPECT_FALSE(a.att);
    EXPECT_FALSE(a.nav);
    EXPECT_EQ(a.fc_frames, 0);

    // Version 2 (the board's IMU watch) keeps the first 32 bits where they were
    ASSERT_TRUE(decodeEpochAid(kE4.data(), kE4.size(), a));
    EXPECT_TRUE(a.present);
    EXPECT_EQ(a.ver, 2);
    EXPECT_EQ(a.src, 0);       // nothing: the IMU refused
    EXPECT_EQ(a.check, 2);     // the pad check failed
    EXPECT_TRUE(a.boost);
    EXPECT_FALSE(a.wide);
    EXPECT_EQ(a.by, 1);
    EXPECT_FALSE(a.planned);
    EXPECT_EQ(a.phase, kPhBoost);
    EXPECT_TRUE(a.att);
    EXPECT_TRUE(a.nav);
    EXPECT_FALSE(a.late);
    EXPECT_EQ(a.fc_frames, 0x3C);
}

TEST(SdrAid, AnOlderEpochHasNoBlock)
{
    // The board's recorded epoch (202 bits, before its checks' marks) and the
    // same with the most marks a 72-byte payload holds (test_sdr_rtcm.cpp's
    // 36): every bit after the marks is padding, so neither shows a block.
    const Bytes s(kSdrStream, kSdrStream + sizeof(kSdrStream));
    std::vector<Bytes> frames;
    for (size_t i = 0; i + 6 <= s.size();)
    {
        const size_t len = 6 + (((s[i + 1] & 3) << 8) | s[i + 2]);
        frames.emplace_back(s.begin() + static_cast<long>(i), s.begin() + static_cast<long>(i + len));
        i += len;
    }
    const Bytes& old_epoch = frames[8];
    EpochAid a;
    ASSERT_TRUE(decodeEpochAid(old_epoch.data(), old_epoch.size(), a));
    EXPECT_FALSE(a.present);

    Bytes payload = payloadOf(old_epoch);
    size_t pos = 202;
    auto put = [&](uint32_t v, int n) {
        for (int i = n - 1; i >= 0; i--, pos++)
        {
            if (pos / 8 >= payload.size()) payload.push_back(0);
            if ((v >> i) & 1U) payload[pos / 8] |= static_cast<uint8_t>(0x80 >> (pos % 8));
        }
    };
    put(2, 6);
    put(36, 6);
    for (int k = 0; k < 36; k++)
    {
        put(static_cast<uint32_t>(k % 3), 2);
        put(static_cast<uint32_t>(k + 1), 6);
        put(1, 2);
    }
    const Bytes marked = frameOf(payload);
    ASSERT_EQ(marked.size(), 3u + 72u + 3u);
    ASSERT_TRUE(decodeEpochAid(marked.data(), marked.size(), a));
    EXPECT_FALSE(a.present);
    sdr_rtcm::TrEpoch e;
    EXPECT_TRUE(sdr_rtcm::decodeTrEpoch(marked.data(), marked.size(), e));

    // A fix, a status and a short epoch are not epochs
    EXPECT_FALSE(decodeEpochAid(frames[0].data(), frames[0].size(), a));   // the fix
    Bytes cut = payloadOf(kE3);
    cut.resize(25);   // 200 bits: short of the sample
    const Bytes short_epoch = frameOf(cut);
    EXPECT_FALSE(decodeEpochAid(short_epoch.data(), short_epoch.size(), a));
}

namespace {

// E0's epoch as the board's encoder writes it with n marks (the reference
// codec's enc_epoch: mark k is system k % 4, PRN k + 1, flags k % 3 + 1)
// and, when `ver` is set, the aiding block after them; ver 0 writes the
// block's other fields alone, as no encoder does.
Bytes epochWithMarks(int n, bool block, uint8_t ver, uint8_t phase, bool late, uint8_t fc_frames)
{
    const Bytes e0 = payloadOf(kE0);
    Bytes p(72, 0);
    size_t pos = 0;
    auto put = [&](uint32_t v, int bits) {
        for (int i = bits - 1; i >= 0; i--, pos++)
            if ((v >> i) & 1U) p[pos / 8] |= static_cast<uint8_t>(0x80 >> (pos % 8));
    };
    for (size_t i = 0; i < 202; i++) put((e0[i / 8] >> (7 - i % 8)) & 1U, 1);   // through the sample
    put(3, 6);                                                                     // ranges left out
    put(static_cast<uint32_t>(n), 6);
    for (int k = 0; k < n; k++)
    {
        put(static_cast<uint32_t>(k % 4), 2);
        put(static_cast<uint32_t>((k + 1) & 63), 6);
        put(static_cast<uint32_t>(k % 3 + 1), 2);
    }
    if (block)
    {
        put(ver, 4);
        put(2, 2);       // src: the board's IMU
        put(0, 2);
        put(1, 1);       // boost
        put(0, 1);
        put(1, 2);       // by the flight computer
        put(0, 1);
        put(phase, 3);
        put(1, 1);       // att
        put(0, 1);
        put(late ? 1 : 0, 1);
        put(0, 5);
        put(fc_frames, 8);
    }
    p.resize((pos + 7) / 8);
    return frameOf(p);
}

}  // namespace

TEST(SdrAid, TheBlockIsFoundAfterAnyMarkCount)
{
    // The block sits after as many marks as the count says, so each count
    // from 0 to 32 puts it somewhere else, ending on the frame's last bit
    // for counts of 1, 5, 9 ...; an epoch without one has at most 7 bits
    // after its marks, and 32 bits there whose version is 0 are no block.
    for (int n = 0; n <= 32; n++)
    {
        SCOPED_TRACE(n);
        const uint8_t ver = static_cast<uint8_t>(1 + n % 15);
        const uint8_t fc = static_cast<uint8_t>(7 * n + 1);
        const Bytes with = epochWithMarks(n, true, ver, static_cast<uint8_t>(n % 8), n % 2 == 1, fc);
        ASSERT_LE(with.size(), 3u + 72u + 3u);
        EXPECT_EQ(with.size(), 6u + (214u + 10u * n + 32u + 7u) / 8u);
        EpochAid a;
        ASSERT_TRUE(decodeEpochAid(with.data(), with.size(), a));
        EXPECT_TRUE(a.present);
        EXPECT_EQ(a.ver, ver);
        EXPECT_EQ(a.src, 2);
        EXPECT_TRUE(a.boost);
        EXPECT_EQ(a.by, 1);
        EXPECT_EQ(a.phase, n % 8);
        EXPECT_TRUE(a.att);
        EXPECT_EQ(a.late, n % 2 == 1);
        EXPECT_EQ(a.fc_frames, fc);
        EXPECT_EQ(a.shift_ns, -499999);
        sdr_rtcm::TrEpoch e;
        EXPECT_TRUE(sdr_rtcm::decodeTrEpoch(with.data(), with.size(), e));
        double latch = 0.0;
        EXPECT_EQ(latchGps(e, a, latch), !a.late);   // a late one stays off the clock line

        const Bytes without = epochWithMarks(n, false, 0, 0, false, 0);
        ASSERT_TRUE(decodeEpochAid(without.data(), without.size(), a));
        EXPECT_FALSE(a.present);
        EXPECT_EQ(a.fc_frames, 0);
        EXPECT_EQ(a.shift_ns, -499999);
        EXPECT_TRUE(latchGps(e, a, latch));

        const Bytes ver0 = epochWithMarks(n, true, 0, 3, true, 0xA5);
        ASSERT_TRUE(decodeEpochAid(ver0.data(), ver0.size(), a));
        EXPECT_FALSE(a.present);
        EXPECT_EQ(a.phase, 0);
        EXPECT_FALSE(a.late);
        EXPECT_EQ(a.fc_frames, 0);
        EXPECT_TRUE(latchGps(e, a, latch));
    }
}

TEST(SdrAid, TheFramerFindsThemInAnySlicing)
{
    // SdrRtcm's framer, the one the driver reads the board with, takes the
    // 4002 frames as any RTCM3 (the FC never reads them off a wire: this is
    // what a log's reader and the board's own recording see).
    std::vector<Bytes> want;
    Bytes stream;
    for (const auto& [sub, f] : allFc())
    {
        want.push_back(f);
        stream.insert(stream.end(), f.begin(), f.end());
    }
    struct Collect : sdr_rtcm::Rtcm3Framer::Sink
    {
        std::vector<Bytes> frames;
        void onFrame(const uint8_t* f, size_t len, size_t) override { frames.emplace_back(f, f + len); }
    };
    for (size_t piece = 1; piece <= 64; piece++)
    {
        for (const Bytes& f : want)
        {
            sdr_rtcm::Rtcm3Framer r;
            Collect c;
            for (size_t i = 0; i < f.size(); i += piece) r.feed(f.data() + i, std::min(piece, f.size() - i), c);
            ASSERT_EQ(c.frames.size(), 1u) << piece;
            EXPECT_EQ(c.frames[0], f);
        }
        sdr_rtcm::Rtcm3Framer r;
        Collect c;
        for (size_t i = 0; i < stream.size(); i += piece)
            r.feed(stream.data() + i, std::min(piece, stream.size() - i), c);
        EXPECT_EQ(c.frames, want) << piece;
        EXPECT_EQ(r.crcFails(), 0u);
        EXPECT_EQ(r.skipped(), 0u);
    }
}

// ── The bench's time line: the board's epochs at their latch ────────────

namespace {

// A 4001 sub-type 1 as the board sends it, its fields after the shift 0 and
// no marks (27 bytes).
Bytes epochFrame(uint16_t week, uint32_t tow_ms, int32_t shift_ns)
{
    Bytes p(27, 0);
    size_t pos = 0;
    auto put = [&](uint64_t v, int n) {
        for (int i = n - 1; i >= 0; i--, pos++)
            if ((v >> i) & 1U) p[pos / 8] |= static_cast<uint8_t>(0x80 >> (pos % 8));
    };
    put(4001, 12);
    put(1, 4);
    put(week, 16);
    put(tow_ms, 30);
    put(static_cast<uint32_t>(shift_ns) & 0xFFFFF, 20);
    return frameOf(p);
}

}  // namespace

TEST(SdrAid, TheBenchStampsComeFromTheBoardsLatches)
{
    // The board latches its measurements on its own clock, a shift_ns (fixed
    // within a boot) off the whole ms its MSMs are tagged with, and its 4001
    // ends here a fixed delay after the latch. Fed at the latch (the tag less
    // the shift), the flight computer's line is the latch's own: a stamp from
    // it is the true GPS time less that delay alone (P1 leaves the delay in:
    // latency_s 0, tq 5). Fed the tag, it would be off by the shift too.
    const int32_t shift_ns = -437123;             // this boot's
    const double delay_s = 0.0042;                // latch to the 4001's last byte here
    const double drift = 17e-6;                   // the FC's crystal
    const double gps0 = 2432 * 604800.0 + 203400.0;
    const int64_t mcu0 = 81234567;
    auto mcuAt = [&](double gps) { return mcu0 + static_cast<int64_t>(std::llround((gps - gps0) * (1.0 + drift) * 1e6)); };

    lc86::GnssTimeSync by_latch, by_tag;
    for (int k = 0; k < 6000; k++)   // a minute of epochs at 100 Hz
    {
        const uint32_t tow_ms = 203400000 + 10 * static_cast<uint32_t>(k);
        const Bytes f = epochFrame(2432, tow_ms, shift_ns);
        sdr_rtcm::TrEpoch e;
        EpochAid aid;
        ASSERT_TRUE(sdr_rtcm::decodeTrEpoch(f.data(), f.size(), e));
        ASSERT_TRUE(decodeEpochAid(f.data(), f.size(), aid));
        ASSERT_EQ(aid.shift_ns, shift_ns);
        double latch = 0.0;
        ASSERT_TRUE(latchGps(e, aid, latch));   // as the driver feeds its line
        const double tag = e.week * 604800.0 + e.tow_ms * 1e-3;
        const int64_t end_us = mcuAt(gps0 + 10 * k * 1e-3 - shift_ns * 1e-9 + delay_s);   // the true latch, then the delay
        by_latch.addSample(latch, end_us);
        by_tag.addSample(tag, end_us);
    }
    ASSERT_TRUE(by_latch.valid());
    lc86::GnssTimeSync::Snapshot s;
    by_latch.snapshot(s);
    GnssClockData line;
    lc86::gnssClockRecord(s, 0u, line);
    GnssClockData tag_line;
    by_tag.snapshot(s);
    lc86::gnssClockRecord(s, 0u, tag_line);

    for (double after : {0.0013, 17.25, 59.9913})
    {
        const double truth = gps0 + after;
        const int64_t now_us = mcuAt(truth);
        double gps = 0.0;
        ASSERT_TRUE(lc86::gnssClockGpsAt(line, now_us, gps));
        EXPECT_NEAR(gps, truth - delay_s, 20e-6) << after;   // the line's fitted drift: microseconds
        const Header h = scriptStamp(gps);
        EXPECT_EQ(h.week, 2432);
        EXPECT_EQ(h.tow_ms, static_cast<uint32_t>(std::llround((truth - delay_s - 2432 * 604800.0) * 1000.0))) << after;
        double by_tag_gps = 0.0;
        ASSERT_TRUE(lc86::gnssClockGpsAt(tag_line, now_us, by_tag_gps));
        EXPECT_NEAR(by_tag_gps - gps, shift_ns * 1e-9, 10e-6) << after;
    }
}

// ── The schedule ────────────────────────────────────────────────────────

namespace {

// One pass of the bench step (main.cpp sdrAidBenchStep): anything due, or
// kKeyLookUs after the last look, then the phase key at the pass's stamp
// (keyAt, worked out only then), then the sub-type with it, sent. 0: nothing
// went.
template <typename KeyAt>
uint8_t benchPass(AidSchedule& s, int64_t& look, int64_t now, KeyAt keyAt, uint32_t& key)
{
    if (!s.anyDue(now) && now < look) return 0;
    look = now + AidSchedule::kKeyLookUs;
    key = keyAt(now);
    const uint8_t sub = s.due(now, key);
    if (sub != 0) s.sent(sub, now, key);
    return sub;
}

struct Sent
{
    int64_t  t;
    uint8_t  sub;
    uint32_t key;
};

// Passes 1 ms apart from `from` to `to`, each on time, one phase key.
void passes(AidSchedule& s, int64_t from, int64_t to, std::vector<Sent>& out)
{
    int64_t look = 0;
    for (int64_t now = from; now < to; now += 1000)
    {
        uint32_t k = 0;
        const uint8_t sub = benchPass(s, look, now, [](int64_t) { return 9u; }, k);
        if (sub != 0) out.push_back({now, sub, k});
    }
}

}  // namespace

TEST(AidSchedule, RatesSpacingAndChanges)
{
    // The bench step's loop: 10 s of passes 1 ms apart, each up to 0.5 ms
    // late, one stall of 5 ms; the phase key from the script at the stamp of
    // each pass that looks, the hotshot scene's launch at 1.1637 s and its
    // burnout at 5.2537 s (off the attitude's 100 ms grid).
    AidSchedule s;
    int64_t look = 0;
    std::mt19937 rng{1600};
    std::uniform_int_distribution<int64_t> late(0, 500);
    const int64_t t0 = 5000000;
    const int64_t tow0_us = 203999000000LL - 3700;   // the GPS time of t0, us of week
    auto towMs = [&](int64_t now) { return static_cast<uint32_t>((tow0_us + (now - t0)) / 1000); };
    auto keyAt = [&](int64_t now) { return scriptPhaseKey(towMs(now)); };

    std::vector<Sent> phase;
    int count[5] = {};
    int64_t last = 0;
    bool any = false;
    double worst_delay_us = 0.0;
    for (int i = 0; i < 10000; i++)
    {
        if (i > 4000 && i < 4005) continue;   // the stall
        const int64_t now = t0 + 1000LL * i + late(rng);
        uint32_t k = 0;
        const uint8_t sub = benchPass(s, look, now, keyAt, k);
        if (sub == 0) continue;
        if (any)
        {
            EXPECT_GE(now - last, AidSchedule::kSpacingUs) << i;
        }
        EXPECT_FALSE(s.anyDue(now)) << "two in one pass at " << i;
        if (sub == kPhase)
        {
            if (!phase.empty() && k != phase.back().key)
            {
                // A change: how long after the first ms of its key it went
                uint32_t c = towMs(now);
                while (scriptPhaseKey(c - 1) == k) c--;
                const double delay_us = static_cast<double>(now - t0) - (c * 1000.0 - static_cast<double>(tow0_us));
                worst_delay_us = std::max(worst_delay_us, delay_us);
            }
            phase.push_back({now, sub, k});
        }
        count[sub]++;
        last = now;
        any = true;
    }

    // Each change, then its three repeats 100 ms apart from 3 ms after it
    // (off the pass the change took), then one a second until the next
    // change: each at or after its time and within a few passes of it (a
    // frame due with it goes first, and a pass on time after one 0.5 ms late
    // is inside the 1 ms spacing: 3.5 ms at most over 120 000 runs of this
    // loop's jitter and alignments).
    int changes = 0;
    for (size_t i = 0; i < phase.size();)
    {
        changes++;
        const int64_t c = phase[i].t;
        size_t j = i + 1;
        for (int n = 1; j < phase.size() && phase[j].key == phase[i].key; j++, n++)
        {
            const int64_t due = c + 3000 + (n <= 3 ? 100000LL * n : 300000LL + 1000000LL * (n - 3));
            EXPECT_GE(phase[j].t - due, 0) << "repeat " << n << " of the change at " << c - t0;
            EXPECT_LT(phase[j].t - due, 6000) << "repeat " << n << " of the change at " << c - t0;
        }
        i = j;
    }
    EXPECT_EQ(changes, 3);   // the first key, the launch, the burnout
    EXPECT_NEAR(count[kAttitude], 100, 1);
    EXPECT_NEAR(count[kNavState], 50, 1);
    EXPECT_EQ(count[kGeometry], 1);   // at the start; the next 10 s after it, past the last pass
    EXPECT_NEAR(count[kPhase], 10 + 3 * changes, 2);
    // Within a look's 10 ms, and the pass that looks (1 ms, up to 0.5 ms
    // late)
    EXPECT_LE(worst_delay_us, AidSchedule::kKeyLookUs + 1500.0);
    EXPECT_GT(worst_delay_us, 0.0);
}

TEST(AidSchedule, EachKeepsToItsOwnPass)
{
    // Passes 1 ms apart and on time for 30 s, one phase key: after the first
    // two (the key, then the attitude it held up), each frame goes on the
    // pass it falls due, at its offset from the attitude's: the state 5 ms,
    // the phase's repeats 3 ms (from the key's pass), the geometry 7 ms.
    AidSchedule s;
    std::vector<Sent> out;
    const int64_t t0 = 1000000;
    passes(s, t0, t0 + 30000000, out);
    ASSERT_GE(out.size(), 2u);
    EXPECT_EQ(out[0].sub, kPhase);
    EXPECT_EQ(out[0].t, t0);
    EXPECT_EQ(out[1].sub, kAttitude);
    EXPECT_EQ(out[1].t, t0 + 1000);
    const int64_t offset[5] = {-1, 0, 5000, 3000, 7000};
    int count[5] = {};
    for (size_t i = 0; i < out.size(); i++)
    {
        count[out[i].sub]++;
        if (i >= 2)
        {
            EXPECT_EQ((out[i].t - t0) % 100000, offset[out[i].sub]) << i << ": sub-type " << int(out[i].sub);
        }
    }
    EXPECT_EQ(count[kAttitude], 300);
    EXPECT_EQ(count[kNavState], 150);
    EXPECT_EQ(count[kPhase], 1 + 3 + 29);   // the key, its three repeats, then one a second
    EXPECT_EQ(count[kGeometry], 3);
}

TEST(AidSchedule, AfterAStallEachGoesOnceInItsOrder)
{
    // Ten seconds on time, then no pass for 315 ms over the attitude's, the
    // state's, the geometry's and the phase's times (10.000, 10.005, 10.007
    // and 10.303 s on): they go one a pass in the order of priority, and the
    // attitude, more than its period late, restarts from its pass: the next
    // 100 ms after it, not on the next pass.
    AidSchedule s;
    std::vector<Sent> out;
    const int64_t t0 = 1000000;
    passes(s, t0, t0 + 10000000, out);
    out.clear();
    passes(s, t0 + 10315000, t0 + 10700000, out);
    ASSERT_GE(out.size(), 4u);
    const uint8_t order[4] = {kAttitude, kNavState, kPhase, kGeometry};
    for (int i = 0; i < 4; i++)
    {
        EXPECT_EQ(out[i].sub, order[i]) << i;
        EXPECT_EQ(out[i].t - t0, 10315000 + 1000 * i) << i;
    }
    std::vector<int64_t> attitude;
    for (const Sent& f : out)
        if (f.sub == kAttitude) attitude.push_back(f.t - t0);
    EXPECT_EQ(attitude, (std::vector<int64_t>{10315000, 10415000, 10515000, 10615000}));
}

TEST(AidSchedule, ALongStallBringsNoBurst)
{
    // Ten seconds on time, then 2 s with no pass over a time of each
    // sub-type: each goes once when the passes come back, then each keeps
    // its rate from there.
    AidSchedule s;
    std::vector<Sent> out;
    passes(s, 1000000, 11000000, out);
    out.clear();
    passes(s, 13000000, 14000000, out);
    int count[5] = {};
    for (const Sent& f : out) count[f.sub]++;
    EXPECT_EQ(count[kAttitude], 10);   // 1 s at 10 Hz, the first at once
    EXPECT_LE(count[kNavState], 6);
    EXPECT_LE(count[kPhase], 1);
    EXPECT_LE(count[kGeometry], 1);
}

// ── What the board says it took ─────────────────────────────────────────

TEST(EchoCount, FollowsTheBoardsCountRoundItsEightBits)
{
    // Two minutes of epochs 10 ms apart, one or two frames out between two
    // (125 a second), the board taking each by the epoch after next: its
    // count wraps every 256, and what it took is what went out, less the
    // frames still on their way.
    EchoCount e;
    uint32_t sent = 0, taken = 0;
    uint32_t seen_sent = 0, seen_taken = 0;
    for (int k = 0; k < 12000; k++)
    {
        e.add(static_cast<uint8_t>(taken), sent, 1000000 + 10000LL * k);
        seen_sent = sent;
        seen_taken = taken;
        taken = sent;
        sent += k % 4 == 0 ? 2 : 1;
    }
    EXPECT_EQ(e.setAside(), 0u);
    EXPECT_EQ(e.of(), seen_sent);
    EXPECT_EQ(e.took(), seen_taken);
    EXPECT_GT(e.took(), 14000u);
}

TEST(EchoCount, SetsAsideWhereTheBoardsCountCannotBeFollowed)
{
    EchoCount e;
    int64_t t = 1000000;
    auto block = [&](uint32_t taken, uint32_t sent, int64_t after_us = 10000) {
        t += after_us;
        e.add(static_cast<uint8_t>(taken), sent, t);
    };
    // Before the first block
    block(5, 6);
    EXPECT_EQ(e.setAside(), 6u);
    // Two out between blocks, the board's count one behind: followed
    block(7, 8);
    block(9, 10);
    EXPECT_EQ(e.took(), 4u);
    EXPECT_EQ(e.of(), 4u);
    // Its epochs stop for 2.5 s (`rx stop`, a scene's end) while 300 more go
    // out and it takes them all: 8 bits would read 45 taken. Set aside.
    block(310, 310, 2500000);
    EXPECT_EQ(e.took(), 4u);
    EXPECT_EQ(e.of(), 4u);
    EXPECT_EQ(e.setAside(), 306u);
    block(311, 312);
    EXPECT_EQ(e.took(), 5u);
    EXPECT_EQ(e.of(), 6u);
    // Its count cleared (`rx fc clear`) while its epochs come: back from 311
    // to 1, which 8 bits read as 202 taken of 2 out. Set aside.
    block(1, 314);
    EXPECT_EQ(e.took(), 5u);
    EXPECT_EQ(e.setAside(), 308u);
    block(3, 316);
    EXPECT_EQ(e.took(), 7u);
    EXPECT_EQ(e.of(), 8u);
    // A reboot: 20 s with no epochs and nothing out, its count back at 0
    block(0, 316, 20000000);
    EXPECT_EQ(e.took(), 7u);
    EXPECT_EQ(e.setAside(), 308u);
    block(2, 318);
    EXPECT_EQ(e.took(), 9u);
    EXPECT_EQ(e.of(), 10u);
    // A gap of 2.1 s (over kGapUs) with a count that would fit: a reboot
    // could be in it all the same
    block(4, 322, 2100000);
    EXPECT_EQ(e.took(), 9u);
    EXPECT_EQ(e.setAside(), 312u);
    block(6, 324);
    EXPECT_EQ(e.took(), 11u);
    EXPECT_EQ(e.of(), 12u);
    // Its epochs held up 0.64 s (its latch ring behind) with 70 out and
    // taken: followed
    block(76, 394, 640000);
    EXPECT_EQ(e.took(), 81u);
    EXPECT_EQ(e.of(), 82u);
    // 120 out in a second, 50 of them lost on the way: its count says so
    block(146, 514, 1000000);
    EXPECT_EQ(e.took(), 151u);
    EXPECT_EQ(e.of(), 202u);
    // Its count up to the slack past what went out is followed; one more is
    // a count that cannot have come from these frames
    block(146 + 2 + EchoCount::kSlack, 516);
    EXPECT_EQ(e.took(), 169u);
    EXPECT_EQ(e.of(), 204u);
    block(164 + 2 + EchoCount::kSlack + 1, 518);
    EXPECT_EQ(e.took(), 169u);
    EXPECT_EQ(e.setAside(), 314u);
    // More out between two blocks than 8 bits tell apart with the slack,
    // even within the gap: 239 is followed, 240 set aside
    block(183 + 239, 518 + 239, 1500000);
    EXPECT_EQ(e.took(), 169u + 239u);
    EXPECT_EQ(e.of(), 204u + 239u);
    block(183 + 239 + 240, 518 + 239 + 240, 1500000);
    EXPECT_EQ(e.took(), 169u + 239u);
    EXPECT_EQ(e.setAside(), 314u + 240u);
}

// ── The flight image's phase (projects/flight_computer/main/
//    sdr_phase_policy.h, main.cpp sdrPhaseStep) ──────────────────────────

namespace {

using sdr_phase::Flight;

// The flags at one pass, with a launch 10 s before it, the burnout 3.9 s
// after the launch and the apogee 8 s after it.
Flight flagsOf(RocketState s, bool burnout, bool apogee, bool canopy, bool sim, bool ignition = false)
{
    Flight f;
    f.state = s;
    f.burnout = burnout;
    f.apogee = apogee;
    f.canopy = canopy;
    f.sim = sim;
    f.ignition = ignition;
    f.now_ms = 1000000;
    f.launch_ms = 990000;
    f.burnout_ms = 993900;
    f.apogee_ms = 998000;
    return f;
}

// What the board does with a phase it reads under its boot defaults
// (`rx fc test off`): takes it unless it is a test value or the script's
// (receiver/v10-link rx_in.c taken()).
bool boardTakes(const Header& h) { return !(h.test || h.src == kSrcScript); }

// n bits at bit pos of a frame's message (after its 3-byte head).
uint32_t bitsOf(const Bytes& f, size_t pos, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++, pos++) v = (v << 1) | ((f[3 + pos / 8] >> (7 - pos % 8)) & 1U);
    return v;
}

// A valid line as the driver logs one: GPS time = gps0_s + its formula
// (GnssClockRecord.h) with the crystal 17 ppm fast.
GnssClockData validLine()
{
    GnssClockData c{};
    c.flags = GCL_VALID | GCL_STARTED;
    c.gps0_s = 2432u * 604800u + 203000u;
    c.mcu0_us = 4123456;
    c.a_s = 0.0021;
    c.t_ref_s = 31.0;
    c.drift = 17.4e-6;
    c.latency_s = 0.0;
    return c;
}

// The bench keeps its schedule in a function-local static on the flight
// loop, the flight image its sender and its ignition cue at file scope
// (main.cpp): constant-initialized, so no guard is taken on the first pass or
// checked on each one after, and nothing runs before the loop. These fail to
// compile otherwise.
[[maybe_unused]] constexpr AidSchedule kBenchSchedule{};
[[maybe_unused]] constexpr sdr_phase::Sender kFlightSender{};
[[maybe_unused]] constexpr sdr_phase::IgnitionCue kFlightCue{};

// A frame's stamp as continuous GPS seconds.
double stampGps(const Header& h)
{
    return h.week * 604800.0 + h.tow_ms * 1e-3 + h.sub_ns * 1e-9;
}

}  // namespace

TEST(SdrPhase, EveryStateAndFlag)
{
    // The P1 spec's map (2.5), first match wins: landed whatever the flags;
    // in flight, the apogee before the burnout before the boost; the pad in
    // every ground state, but a boost in those a launch leaves while the
    // IMU's ignition cue holds (not MAG_CALIBRATION's, which no launch
    // leaves; the cue changes nothing in flight or landed). The canopy only
    // in a descent; since_ms from the launch, burnout and apogee latches;
    // 65535 on the pad, in the cue's boost and after landing. Every frame
    // real (src 1) but in a sim, held 2.5 s, tq 5.
    const RocketState states[] = {INITIALIZATION, READY, PRELAUNCH, INFLIGHT, LANDED, MAG_CALIBRATION};
    for (RocketState s : states)
    {
        for (int m = 0; m < 32; m++)
        {
            const bool burnout = m & 1, apogee = m & 2, canopy = m & 4, sim = m & 8, ignition = m & 16;
            const PhaseMsg p = sdr_phase::phaseOf(flagsOf(s, burnout, apogee, canopy, sim, ignition));
            SCOPED_TRACE(testing::Message() << "state " << int(s) << " flags " << m);
            uint8_t ph = kPhPad, loops = kLoopsNone;
            uint32_t since = 65535;
            bool can = false;
            if (s == LANDED)
            {
                ph = kPhLanded;
            }
            else if (s == INFLIGHT && apogee)
            {
                ph = kPhDescent;
                can = canopy;
                since = 2000;
            }
            else if (s == INFLIGHT && burnout)
            {
                ph = kPhCoast;
                loops = kLoopsQuiet;
                since = 6100;
            }
            else if (s == INFLIGHT)
            {
                ph = kPhBoost;
                loops = kLoopsBoost;
                since = 10000;
            }
            else if (ignition && s != MAG_CALIBRATION)
            {
                ph = kPhBoost;
                loops = kLoopsBoost;
            }
            EXPECT_EQ(p.phase, ph);
            EXPECT_NE(p.phase, kPhUnknown);
            EXPECT_EQ(p.loops, loops);
            EXPECT_EQ(p.canopy, can);
            EXPECT_EQ(p.since_ms, since);
            EXPECT_EQ(p.hold_ms, 2500u);
            EXPECT_EQ(p.hold_ms, kPhaseHoldMs);
            EXPECT_EQ(p.h.src, kSrcOwn);
            EXPECT_EQ(p.h.test, sim);
            EXPECT_EQ(p.h.tq, 5);
            EXPECT_EQ(p.h.tq, kLineTq);
            EXPECT_EQ(boardTakes(p.h), !sim);
        }
    }
}

TEST(SdrPhase, SinceSaturatesAndNeverWraps)
{
    Flight f = flagsOf(INFLIGHT, false, false, false, false);
    struct Row { uint32_t now, from, since; };
    const Row rows[] = {
        {1000000, 1000000, 0},          {1000001, 1000000, 1},          {1065534, 1000000, 65534},
        {1065535, 1000000, 65535},      {1070000, 1000000, 65535},      {999999, 1000000, 0},   // a start not yet passed
        {0x00000100u, 0xFFFFFF00u, 512},                                                     // across ms's wrap
        {600000000, 1000, 65535},
    };
    for (const Row& r : rows)
    {
        f.now_ms = r.now;
        f.launch_ms = r.from;
        EXPECT_EQ(sdr_phase::phaseOf(f).since_ms, r.since) << r.now << " " << r.from;
        EXPECT_EQ(sdr_phase::sinceMs(r.now, r.from), r.since);
    }
    // The coast and the descent count from their own latches
    f = flagsOf(INFLIGHT, true, false, false, false);
    f.now_ms = f.burnout_ms + 123;
    EXPECT_EQ(sdr_phase::phaseOf(f).since_ms, 123u);
    f = flagsOf(INFLIGHT, true, true, true, false);
    f.now_ms = f.apogee_ms + 70000;
    EXPECT_EQ(sdr_phase::phaseOf(f).since_ms, 65535u);
}

TEST(SdrPhase, AFrameTheBoardTakes)
{
    // Each phase stamped and encoded: a 22-byte 4002 sub-type 3 that decodes
    // to what was sent, its hold the code 125 (2500 ms in 20 ms steps), its
    // stamp the GPS instant to the nearest ms with the rest in ns.
    const double gps = 2432 * 604800.0 + 204017.2496123;
    const Flight cases[] = {
        flagsOf(READY, false, false, false, false),   flagsOf(INFLIGHT, false, false, false, false),
        flagsOf(INFLIGHT, true, false, false, false), flagsOf(INFLIGHT, true, true, true, false),
        flagsOf(LANDED, true, true, true, false),     flagsOf(INFLIGHT, false, false, false, true),
    };
    uint8_t seq = 254;
    for (const Flight& f : cases)
    {
        const PhaseMsg p = sdr_phase::stamped(sdr_phase::phaseOf(f), gps, seq);
        const Bytes b = enc(p);
        ASSERT_EQ(b.size(), kPhaseFrame);
        ASSERT_EQ(b.size(), 22u);
        EXPECT_EQ(fcSubType(b.data(), b.size()), kPhase);
        EXPECT_EQ(bitsOf(b, 118, 8), 125u);   // the hold's code
        PhaseMsg d;
        ASSERT_TRUE(decode(b.data(), b.size(), d));
        EXPECT_EQ(d.h.week, 2432);
        EXPECT_EQ(d.h.tow_ms, 204017250u);
        EXPECT_EQ(d.h.sub_ns, p.h.sub_ns);
        EXPECT_NEAR(d.h.sub_ns, -387700, 250);   // kept, not zeroed as a script frame's (a double at 1.5e9 s
                                                 // holds the ns to a quarter of a microsecond)
        EXPECT_EQ(d.h.seq, seq);
        EXPECT_EQ(d.h.src, kSrcOwn);
        EXPECT_EQ(d.h.test, f.sim);
        EXPECT_EQ(d.h.tq, kLineTq);
        EXPECT_EQ(d.phase, p.phase);
        EXPECT_EQ(d.loops, p.loops);
        EXPECT_EQ(d.canopy, p.canopy);
        EXPECT_EQ(d.since_ms, p.since_ms);
        EXPECT_EQ(d.hold_ms, 2500u);
        EXPECT_EQ(boardTakes(d.h), !f.sim);
        EXPECT_EQ(enc(d), b);
        seq++;   // 254, 255, 0, ...
    }
}

TEST(SdrPhase, TheKeyIsWhatTheBoardActsOn)
{
    // A change of the phase, the loops, the canopy or the test flag is one;
    // the time in the phase, the stamp, seq, src, tq and the hold are not.
    const PhaseMsg base = sdr_phase::phaseOf(flagsOf(INFLIGHT, true, true, false, false));
    const uint32_t k0 = phaseKey(base);
    EXPECT_EQ(k0, uint32_t{kPhDescent} | uint32_t{kLoopsNone} << 3);
    PhaseMsg p = base;
    p.phase = kPhLanded;
    EXPECT_NE(phaseKey(p), k0);
    p = base;
    p.loops = kLoopsQuiet;
    EXPECT_NE(phaseKey(p), k0);
    p = base;
    p.canopy = true;
    EXPECT_NE(phaseKey(p), k0);
    p = base;
    p.h.test = true;
    EXPECT_NE(phaseKey(p), k0);
    p = base;
    p.since_ms = 12345;
    p.h = stampAt(2433, 17.0);
    p.h.test = base.h.test;   // (a Header's default is a test)
    p.h.seq = 77;
    p.h.src = kSrcFusedBoard;
    p.h.tq = 2;
    p.hold_ms = 5000;
    EXPECT_EQ(phaseKey(p), k0);
    // Every message the map gives has its own key
    // (the cue's boost is the launch's: one key, so INFLIGHT takes it over
    // with no frame of its own)
    EXPECT_EQ(phaseKey(sdr_phase::phaseOf(flagsOf(PRELAUNCH, false, false, false, false, true))),
              phaseKey(sdr_phase::phaseOf(flagsOf(INFLIGHT, false, false, false, false, true))));
    std::vector<uint32_t> keys;
    for (RocketState s : {READY, INFLIGHT, LANDED})
        for (int m = 0; m < 32; m++)
        {
            const PhaseMsg q = sdr_phase::phaseOf(flagsOf(s, m & 1, m & 2, m & 4, m & 8, m & 16));
            const uint32_t k = phaseKey(q);
            keys.push_back(k);
            EXPECT_EQ(k, uint32_t{q.phase} | uint32_t{q.loops} << 3 | uint32_t{q.canopy} << 5 |
                             uint32_t{q.h.test} << 6);
        }
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    EXPECT_EQ(keys.size(), 2u * (1 + 1 + 1 + 2 + 1));   // pad, boost, coast, descent (canopy or not), landed; x sim
}

TEST(SdrAid, AGpsStampIsStampAtWithTheWeekSplitOff)
{
    // The same arithmetic as the script's stamp, the ns kept
    std::mt19937_64 rng{4002};
    std::uniform_real_distribution<double> u(0.0, 604800.0);
    for (int i = 0; i < 2000; i++)
    {
        const double gps = 2432 * 604800.0 + u(rng);
        const int week = static_cast<int>(std::floor(gps / 604800.0));
        const Header a = gpsStamp(gps);
        const Header b = stampAt(week, gps - week * 604800.0);
        ASSERT_EQ(a.week, b.week);
        ASSERT_EQ(a.tow_ms, b.tow_ms);
        ASSERT_EQ(a.sub_ns, b.sub_ns);
        const Header s = scriptStamp(gps);
        ASSERT_EQ(s.week, a.week);
        ASSERT_EQ(s.tow_ms, a.tow_ms);
        ASSERT_EQ(s.sub_ns, 0);
    }
    // The week's last half ms rounds to the next week's 0 (a double at 1.5e9 s
    // holds the ns to a quarter of a microsecond)
    const Header r = gpsStamp(2433 * 604800.0 - 0.0004);
    EXPECT_EQ(r.week, 2433);
    EXPECT_EQ(r.tow_ms, 0u);
    EXPECT_NEAR(r.sub_ns, -400000, 250);
    const Header w = gpsStamp(2432 * 604800.0 + 204000.1234567);
    EXPECT_EQ(w.week, 2432);
    EXPECT_EQ(w.tow_ms, 204000123u);
    EXPECT_NEAR(w.sub_ns, 456700, 250);
}

TEST(AidSchedule, ThePhaseAloneAtItsOwnTimes)
{
    // The flight image's schedule (the phase alone), 120 s of passes 1 ms
    // apart: never another sub-type; each change on the pass it appears,
    // then 103, 203 and 303 ms after it, then every second; and after a
    // stall of 2.5 s one frame when the passes come back, then a second
    // after that: no burst.
    AidSchedule s(1u << kPhase);
    const int64_t t0 = 3000000;
    // The key from each time on (ms after t0): the pad, a launch, its
    // burnout, the apogee, the canopy 1 ms after the apogee's second repeat,
    // the landing
    const std::vector<std::pair<int64_t, uint32_t>> keys = {
        {0, 1}, {60000, 2 | 2 << 3}, {63900, 3 | 1 << 3}, {80000, 4}, {80204, 4 | 1 << 5}, {110000, 5}};
    auto keyAt = [&](int64_t ms) {
        uint32_t k = 0;
        for (const auto& [from, key] : keys)
            if (ms >= from) k = key;
        return k;
    };
    const int64_t stall_from = 112500, stall_to = 115000;   // ms after t0: no passes
    std::vector<int64_t> sent;
    int64_t last = -1;
    for (int64_t ms = 0; ms < 120000; ms++)
    {
        if (ms >= stall_from && ms < stall_to) continue;
        const int64_t now = t0 + ms * 1000;
        const uint32_t key = keyAt(ms);
        const uint8_t sub = s.due(now, key);
        if (sub == 0) continue;
        ASSERT_EQ(sub, kPhase) << ms;
        if (last >= 0)
        {
            EXPECT_GE(now - last, AidSchedule::kSpacingUs);
        }
        s.sent(sub, now, key);
        EXPECT_EQ(s.due(now, key), 0) << "two in one pass at " << ms;
        sent.push_back(ms);
        last = now;
    }
    std::vector<int64_t> want;
    for (size_t i = 0; i < keys.size(); i++)
    {
        const int64_t c = keys[i].first;
        const int64_t end = i + 1 < keys.size() ? keys[i + 1].first : 120000;
        std::vector<int64_t> seg = {c};
        for (int64_t t : {c + 103, c + 203, c + 303})
            if (t < end) seg.push_back(t);
        for (int64_t t = c + 1303; t < end; t += 1000) seg.push_back(t);
        for (int64_t t : seg)
        {
            if (t >= stall_from && t < stall_to) continue;
            if (t >= stall_to) break;
            want.push_back(t);
        }
    }
    // After the stall: once at its end, then every second from there
    for (int64_t t = stall_to; t < 120000; t += 1000) want.push_back(t);
    EXPECT_EQ(sent, want);
}

TEST(AidSchedule, APhaseChangeWaitsOnlyForTheSpacing)
{
    // Passes closer than 1 ms: a change half a ms after a frame goes on the
    // first pass a ms after it
    AidSchedule s(1u << kPhase);
    EXPECT_EQ(s.due(0, 1), kPhase);   // the first call sends
    s.sent(kPhase, 0, 1);
    EXPECT_EQ(s.due(500, 2), 0);
    EXPECT_EQ(s.due(999, 2), 0);
    EXPECT_EQ(s.due(1000, 2), kPhase);
    s.sent(kPhase, 1000, 2);
    EXPECT_FALSE(s.anyDue(1001));
    EXPECT_FALSE(s.anyDue(103999));
    EXPECT_TRUE(s.anyDue(104000));   // its first repeat, 103 ms after
}

TEST(SdrPhase, NothingWithoutAValidLineMovedInTheLast600s)
{
    sdr_phase::Sender tx;
    const Flight f = flagsOf(READY, false, false, false, false);
    uint8_t b[kMaxFrame];
    const GnssClockData line = validLine();
    const int64_t t = 700000000;   // us
    // No valid line (before the board's first 10 s): nothing, and nothing
    // used up
    GnssClockData none = line;
    none.flags = GCL_STARTED;
    for (int64_t now = t - 5000000; now < t; now += 1000)
    {
        tx.noteLine(none, now - 1000);
        ASSERT_EQ(tx.frame(now, f, b, sizeof(b)), 0u);
    }
    EXPECT_EQ(tx.seq(), 0);
    // A valid line that last moved 600 s ago or more: nothing
    tx.noteLine(line, t - sdr_phase::kLineHoldUs);
    EXPECT_EQ(tx.frame(t, f, b, sizeof(b)), 0u);
    tx.noteLine(line, t - sdr_phase::kLineHoldUs - 1);
    EXPECT_EQ(tx.frame(t, f, b, sizeof(b)), 0u);
    // A buffer too short for a frame gets nothing
    tx.noteLine(line, t);
    EXPECT_EQ(tx.frame(t, f, b, kPhaseFrame - 1), 0u);
    EXPECT_EQ(tx.seq(), 0);
    // Then 2 s with a line moved within the 600 s (the first frame at once
    // and its repeats), 3 s past them (the board's epochs stopped long
    // ago: nothing), 2 s within them again (the second missed goes at once,
    // then one a second from there)
    std::vector<int64_t> sent;
    for (int64_t now = t; now < t + 7000000; now += 1000)
    {
        const bool stale = now >= t + 2000000 && now < t + 5000000;
        tx.noteLine(line, stale ? now - sdr_phase::kLineHoldUs : now - sdr_phase::kLineHoldUs + 1);
        if (tx.frame(now, f, b, sizeof(b)) == 0) continue;
        tx.sent(now);
        sent.push_back((now - t) / 1000);
    }
    EXPECT_EQ(sent, (std::vector<int64_t>{0, 103, 203, 303, 1303, 5000, 6000}));
    EXPECT_EQ(tx.seq(), 7);
}

TEST(SdrPhase, AWholeFlight)
{
    // The flight image's sender pass by pass, 1 ms apart, on a valid line
    // noted once a second (as the driver hands it on): 300 s on the pad
    // (READY), a launch, the
    // burnout at T+3.9 s, the apogee at T+20 s, the deployment at T+21 s,
    // landed at T+90 s, 10 s more. The TX ring is full on the burnout's
    // first try.
    const GnssClockData line = validLine();
    const int64_t t0 = 50000000;   // us: the first pass
    const int64_t T = t0 + 300000000;
    const int64_t burnout = T + 3900000, apogee = T + 20000000, deploy = T + 21000000, landed = T + 90000000;
    const int64_t end = landed + 10000000;
    sdr_phase::Sender tx;
    struct Frame { int64_t now; PhaseMsg p; };
    std::vector<Frame> out;
    bool refused = false;
    for (int64_t now = t0; now < end; now += 1000)
    {
        Flight f;
        f.state = now >= landed ? LANDED : (now >= T ? INFLIGHT : READY);
        f.burnout = now >= burnout;
        f.apogee = now >= apogee;
        f.canopy = now >= deploy;
        f.now_ms = static_cast<uint32_t>(now / 1000);
        f.launch_ms = static_cast<uint32_t>(T / 1000);
        f.burnout_ms = static_cast<uint32_t>(burnout / 1000);
        f.apogee_ms = static_cast<uint32_t>(apogee / 1000);
        if ((now - t0) % 1000000 == 0) tx.noteLine(line, now);
        uint8_t b[kPhaseFrame];
        const size_t len = tx.frame(now, f, b, sizeof(b));
        if (len == 0) continue;
        ASSERT_EQ(len, kPhaseFrame);
        if (now == burnout && !refused)
        {
            refused = true;   // no room: given again on the next pass
            continue;
        }
        PhaseMsg d;
        ASSERT_TRUE(decode(b, len, d));
        out.push_back({now, d});
        tx.sent(now);
    }
    ASSERT_TRUE(refused);
    ASSERT_GT(out.size(), 256u);   // seq goes round

    // The first frame on the first pass; the boost on the pass INFLIGHT
    // appears; the burnout's a pass late (its first try found no room)
    auto firstOf = [&](uint8_t ph) {
        for (const Frame& fr : out)
            if (fr.p.phase == ph) return fr.now;
        return int64_t{-1};
    };
    EXPECT_EQ(out.front().now, t0);
    EXPECT_EQ(firstOf(kPhBoost), T);
    EXPECT_EQ(firstOf(kPhCoast), burnout + 1000);
    EXPECT_EQ(firstOf(kPhDescent), apogee);
    EXPECT_EQ(firstOf(kPhLanded), landed);

    double last_gps = 0.0;
    int64_t seg_start = -1;
    uint32_t seg_key = 0xFFFFFFFF;
    int changes = 0;
    for (size_t i = 0; i < out.size(); i++)
    {
        const Frame& fr = out[i];
        const PhaseMsg& p = fr.p;
        SCOPED_TRACE(testing::Message() << "frame " << i << " at " << (fr.now - T) / 1000 << " ms from the launch");
        // seq one up each frame, round its 8 bits
        EXPECT_EQ(p.h.seq, static_cast<uint8_t>(i));
        // The vehicle's own values, taken by the board as they are
        EXPECT_EQ(p.h.src, kSrcOwn);
        EXPECT_FALSE(p.h.test);
        EXPECT_EQ(p.h.tq, kLineTq);
        EXPECT_EQ(p.hold_ms, 2500u);
        EXPECT_TRUE(boardTakes(p.h));
        // What the flags said at its pass
        uint8_t ph = kPhPad, loops = kLoopsNone;
        bool canopy = false;
        uint32_t since = 65535;
        if (fr.now >= landed) ph = kPhLanded;
        else if (fr.now >= apogee) { ph = kPhDescent; canopy = fr.now >= deploy; since = static_cast<uint32_t>(std::min<int64_t>((fr.now - apogee) / 1000, 65535)); }
        else if (fr.now >= burnout) { ph = kPhCoast; loops = kLoopsQuiet; since = static_cast<uint32_t>((fr.now - burnout) / 1000); }
        else if (fr.now >= T) { ph = kPhBoost; loops = kLoopsBoost; since = static_cast<uint32_t>((fr.now - T) / 1000); }
        EXPECT_EQ(p.phase, ph);
        EXPECT_EQ(p.loops, loops);
        EXPECT_EQ(p.canopy, canopy);
        EXPECT_EQ(p.since_ms, since);
        // Its stamp is the line's GPS time at its pass, to the ns field's
        // rounding, and the stamps only go forward
        double want = 0.0;
        ASSERT_TRUE(lc86::gnssClockGpsAt(line, fr.now, want));
        EXPECT_NEAR(stampGps(p.h), want, 1e-6);
        EXPECT_GT(stampGps(p.h), last_gps);
        last_gps = stampGps(p.h);
        // Each change, then 103, 203 and 303 ms after it, then every second
        const uint32_t key = phaseKey(p);
        if (key != seg_key)
        {
            seg_key = key;
            seg_start = fr.now;
            changes++;
            continue;
        }
        const int64_t dt = (fr.now - seg_start) / 1000;
        const int64_t prev = (out[i - 1].now - seg_start) / 1000;
        const int64_t next_want = prev == 0 ? 103 : (prev < 303 ? prev + 100 : prev + 1000);
        EXPECT_EQ(dt, next_want);
    }
    EXPECT_EQ(changes, 6);   // pad, boost, coast, descent, descent under the canopy, landed

    // Four frames in the boost's 3.9 s and the coast's first second: the
    // change and its three repeats, then one a second
    int boost = 0;
    for (const Frame& fr : out) boost += fr.p.phase == kPhBoost;
    EXPECT_EQ(boost, 4 + 3);   // T, +103, +203, +303, +1303, +2303, +3303
}

TEST(SdrPhase, ASimFliesTestFrames)
{
    // A sim's flight (or a rig replay's) is sent as test frames, which the
    // board takes only under `rx fc test on`; its start and its end are
    // changes, sent at once.
    const GnssClockData line = validLine();
    sdr_phase::Sender tx;
    uint8_t b[kPhaseFrame];
    Flight f = flagsOf(READY, false, false, false, false);
    const int64_t t = 90000000;
    tx.noteLine(line, t);
    ASSERT_GT(tx.frame(t, f, b, sizeof(b)), 0u);
    tx.sent(t);
    f.sim = true;   // the sim starts on the pad
    ASSERT_GT(tx.frame(t + 1000, f, b, sizeof(b)), 0u);
    PhaseMsg d;
    ASSERT_TRUE(decode(b, kPhaseFrame, d));
    EXPECT_EQ(d.phase, kPhPad);
    EXPECT_TRUE(d.h.test);
    EXPECT_FALSE(boardTakes(d.h));
    tx.sent(t + 1000);
    f.state = INFLIGHT;   // its launch
    ASSERT_GT(tx.frame(t + 2000, f, b, sizeof(b)), 0u);
    ASSERT_TRUE(decode(b, kPhaseFrame, d));
    EXPECT_EQ(d.phase, kPhBoost);
    EXPECT_EQ(d.loops, kLoopsBoost);
    EXPECT_TRUE(d.h.test);
    EXPECT_FALSE(boardTakes(d.h));
}

TEST(SdrPhase, ARestartOfTheLineDoesNotStopThePhase)
{
    // The drain hands the sender every record the driver gives (main.cpp):
    // a valid line, then a restart of it in the boost (five epochs off it:
    // records that are not valid for the 10 s the new history takes), then
    // the new valid line. The phase goes on at its pace through the restart,
    // stamped off the old line, and off the new one from the first frame
    // after it came; the stamps stay tq 5 (the line is never 100 s old).
    const GnssClockData a = validLine();
    GnssClockData b = validLine();
    b.a_s = -0.0137;   // 15.8 ms from a's at any instant: told apart
    b.seq = a.seq + 6;
    GnssClockData restart = b;
    restart.flags = GCL_STARTED;
    const int64_t t0 = 30000000;
    const int64_t T = t0 + 5000000;   // the launch
    const int64_t burnout = T + 15500000;
    const int64_t restart_from = T + 1000000, valid_from = T + 12000000;
    sdr_phase::Sender tx;
    struct Frame { int64_t now; PhaseMsg p; };
    std::vector<Frame> out;
    for (int64_t now = t0; now < T + 20000000; now += 1000)
    {
        if ((now - t0) % 1000000 == 0)
            tx.noteLine(now < restart_from ? a : (now < valid_from ? restart : b), now);
        Flight f;
        f.state = now >= T ? INFLIGHT : READY;
        f.burnout = now >= burnout;
        f.now_ms = static_cast<uint32_t>(now / 1000);
        f.launch_ms = static_cast<uint32_t>(T / 1000);
        f.burnout_ms = static_cast<uint32_t>(burnout / 1000);
        uint8_t buf[kPhaseFrame];
        const size_t len = tx.frame(now, f, buf, sizeof(buf));
        if (len == 0) continue;
        PhaseMsg d;
        ASSERT_TRUE(decode(buf, len, d));
        out.push_back({now, d});
        tx.sent(now);
    }
    std::vector<int64_t> boost;
    for (const Frame& fr : out)
    {
        SCOPED_TRACE(testing::Message() << "at " << (fr.now - T) / 1000 << " ms from the launch");
        double want = 0.0;
        ASSERT_TRUE(lc86::gnssClockGpsAt(fr.now < valid_from ? a : b, fr.now, want));
        EXPECT_NEAR(stampGps(fr.p.h), want, 1e-6);
        EXPECT_EQ(fr.p.h.tq, kLineTq);
        if (fr.p.phase == kPhBoost) boost.push_back((fr.now - T) / 1000);
    }
    // The boost's change and repeats, then every second to the burnout,
    // none missed in the restart (1 to 12 s)
    std::vector<int64_t> want = {0, 103, 203, 303};
    for (int64_t ms = 1303; ms < 15500; ms += 1000) want.push_back(ms);
    EXPECT_EQ(boost, want);
    double d_ab = 0.0, d_bb = 0.0;
    ASSERT_TRUE(lc86::gnssClockGpsAt(a, valid_from, d_ab));
    ASSERT_TRUE(lc86::gnssClockGpsAt(b, valid_from, d_bb));
    EXPECT_GT(std::fabs(d_ab - d_bb), 0.010);
}

TEST(SdrPhase, AHeldLinesStampsSayItIsOld)
{
    // One valid line, never again (the board's epochs stopped): tq 5 while
    // it moved within 100 s, 6 after, nothing from 600 s
    const GnssClockData line = validLine();
    const Flight f = flagsOf(READY, false, false, false, false);
    sdr_phase::Sender tx;
    const int64_t t = 10000000;
    tx.noteLine(line, t);
    int fresh = 0, held = 0;
    int64_t last = -1;
    for (int64_t now = t; now < t + sdr_phase::kLineHoldUs + 3000000; now += 1000)
    {
        uint8_t b[kPhaseFrame];
        const size_t len = tx.frame(now, f, b, sizeof(b));
        if (len == 0) continue;
        PhaseMsg d;
        ASSERT_TRUE(decode(b, len, d));
        const int64_t age = now - t;
        EXPECT_EQ(d.h.tq, age <= sdr_phase::kLineTqFreshUs ? kLineTq : sdr_phase::kHeldLineTq) << age;
        (age <= sdr_phase::kLineTqFreshUs ? fresh : held)++;
        last = age;
        tx.sent(now);
    }
    EXPECT_EQ(fresh, 4 + 99);    // 0, 103, 203, 303 ms, then 1.303 ... 99.303 s
    EXPECT_EQ(held, 500);        // 100.303 ... 599.303 s
    EXPECT_LT(last, sdr_phase::kLineHoldUs);
    EXPECT_EQ(sdr_phase::kHeldLineTq, 6);
    EXPECT_EQ(sdr_phase::kLineTqFreshUs, 100000000);
}

TEST(IgnitionCue, FifteenTicksOverTwoGThenHeldASecond)
{
    using sdr_phase::IgnitionCue;
    EXPECT_EQ(IgnitionCue::kMs2, 20.0f);   // the launch detector's first bar (TR_KinematicChecks)
    EXPECT_EQ(IgnitionCue::kCount, 15);
    EXPECT_EQ(IgnitionCue::kHoldMs, 1000u);
    IgnitionCue c;
    uint32_t ms = 5000;
    auto run = [&](int n, float acc) {
        for (int i = 0; i < n; i++, ms++) c.step(ms, acc);
    };
    // At rest on the pad, 1 g, for a minute: never
    for (int i = 0; i < 60000; i++, ms++)
    {
        c.step(ms, 9.81f);
        ASSERT_FALSE(c.on(ms));
    }
    // 14 ticks over the bar, then one at it (not over): a bump, never
    run(14, 25.0f);
    EXPECT_FALSE(c.on(ms - 1));
    run(1, 20.0f);
    EXPECT_FALSE(c.on(ms - 1));
    // A stale IMU (the caller's 0) and a NaN break a run as a low tick does
    run(14, 25.0f);
    run(1, 0.0f);
    run(14, 25.0f);
    run(1, std::numeric_limits<float>::quiet_NaN());
    run(14, 25.0f);
    EXPECT_FALSE(c.on(ms - 1));
    // The 15th in a row: on at that tick
    run(1, 25.0f);
    EXPECT_TRUE(c.on(ms - 1));
    // Through a boost's dips under the bar (each resets the run): on
    for (int k = 0; k < 20; k++)
    {
        run(5, 15.0f);
        for (int i = 0; i < 30; i++, ms++)
        {
            c.step(ms, 45.0f);
            ASSERT_TRUE(c.on(ms));
        }
    }
    // The run's end: held for 1 s from its last tick, then off
    const uint32_t last = ms - 1;
    for (; ms < last + IgnitionCue::kHoldMs; ms++)
    {
        c.step(ms, 9.81f);
        ASSERT_TRUE(c.on(ms)) << ms - last;
    }
    c.step(ms, 9.81f);
    EXPECT_FALSE(c.on(ms));
    ms++;
    // Not stepped (MAG_CALIBRATION skips the flight-logic's checks): it
    // lapses all the same
    run(15, 25.0f);
    EXPECT_TRUE(c.on(ms - 1));
    EXPECT_TRUE(c.on(ms - 1 + IgnitionCue::kHoldMs - 1));
    EXPECT_FALSE(c.on(ms - 1 + IgnitionCue::kHoldMs));
    // Across 32-bit ms's wrap (49.7 days up)
    IgnitionCue w;
    uint32_t m = 0xFFFFFFF8u;
    for (int i = 0; i < 15; i++, m++) w.step(m, 25.0f);
    EXPECT_TRUE(w.on(m - 1));
    EXPECT_TRUE(w.on(m - 1 + 999));
    EXPECT_FALSE(w.on(m - 1 + 1000));
    // A cue that lapsed while stepped stays off, even at its last tick's ms
    // (where 32-bit ms come round to after 49.7 days)
    const uint32_t last_w = m - 1;
    for (int i = 0; i < 2000; i++, m++) w.step(m, 9.81f);
    EXPECT_FALSE(w.on(m - 1));
    EXPECT_FALSE(w.on(last_w));
}

TEST(SdrPhase, TheCueGivesTheBoostBeforeTheLaunchFlag)
{
    // Pass by pass, 1 ms apart, as main.cpp runs them: the phase sent off the
    // cue as the last flight-logic tick left it, then this tick's step. On
    // READY: a 20 ms bump at 3 g at P (a false cue), then the ignition at T,
    // 5 g with a dip to 1.5 g at T+40..45 ms, the launch flag at T+265 ms
    // (the barometer-confirmed path's slowest), the burnout latch at
    // T+2.051 s (0.5 g of drag from T+2 s).
    const GnssClockData line = validLine();
    const int64_t t0 = 60000000;
    const int64_t P = t0 + 5000000, T = t0 + 20000000;
    const int64_t launch = T + 265000, burnout = T + 2051000, end = T + 4000000;
    sdr_phase::Sender tx;
    sdr_phase::IgnitionCue cue;
    struct Frame { int64_t now; PhaseMsg p; };
    std::vector<Frame> out;
    for (int64_t now = t0; now < end; now += 1000)
    {
        if ((now - t0) % 1000000 == 0) tx.noteLine(line, now);
        const uint32_t now_ms = static_cast<uint32_t>(now / 1000);
        Flight f;
        f.state = now >= launch ? INFLIGHT : READY;
        f.burnout = now >= burnout;
        f.now_ms = now_ms;
        f.ignition = cue.on(now_ms);
        f.launch_ms = static_cast<uint32_t>(launch / 1000);
        f.burnout_ms = static_cast<uint32_t>(burnout / 1000);
        uint8_t b[kPhaseFrame];
        const size_t len = tx.frame(now, f, b, sizeof(b));
        if (len > 0)
        {
            PhaseMsg d;
            ASSERT_TRUE(decode(b, len, d));
            out.push_back({now, d});
            tx.sent(now);
        }
        float acc = 9.81f;
        if (now >= P && now < P + 20000) acc = 29.4f;
        if (now >= T && now < T + 2000000) acc = (now >= T + 40000 && now < T + 46000) ? 14.7f : 49.0f;
        if (now >= T + 2000000) acc = 4.9f;
        cue.step(now_ms, acc);
    }
    auto ms = [](int64_t a, int64_t b) { return (a - b) / 1000; };
    // The false cue: boost 15 ms after the bump began, its repeats, the pad
    // again at once 1 s after the bump's last tick (P+19 ms)
    std::vector<std::pair<int64_t, uint8_t>> bump;
    for (const Frame& fr : out)
        if (fr.now >= P && fr.now < P + 1400000) bump.push_back({ms(fr.now, P), fr.p.phase});
    EXPECT_EQ(bump, (std::vector<std::pair<int64_t, uint8_t>>{
                        {15, kPhBoost}, {118, kPhBoost}, {218, kPhBoost}, {318, kPhBoost},
                        {1019, kPhPad}, {1122, kPhPad}, {1222, kPhPad}, {1322, kPhPad}}));
    // The flight: boost 15 ms after the ignition, 250 ms before the launch
    // flag; the launch flag sends nothing of its own (the same key), its
    // repeats and seconds go on from the cue's change, and since_ms counts
    // from the launch once it latched (65535 before: no latch yet)
    struct Want { int64_t ms; uint8_t phase; uint32_t since; };
    const std::vector<Want> want = {{15, kPhBoost, 65535},  {118, kPhBoost, 65535}, {218, kPhBoost, 65535},
                                    {318, kPhBoost, 53},    {1318, kPhBoost, 1053}, {2051, kPhCoast, 0},
                                    {2154, kPhCoast, 103},  {2254, kPhCoast, 203},  {2354, kPhCoast, 303},
                                    {3354, kPhCoast, 1303}};
    std::vector<Frame> flight;
    for (const Frame& fr : out)
        if (fr.now >= T) flight.push_back(fr);
    ASSERT_EQ(flight.size(), want.size());
    for (size_t i = 0; i < want.size(); i++)
    {
        SCOPED_TRACE(testing::Message() << "frame " << i);
        EXPECT_EQ(ms(flight[i].now, T), want[i].ms);
        EXPECT_EQ(flight[i].p.phase, want[i].phase);
        EXPECT_EQ(flight[i].p.loops, want[i].phase == kPhBoost ? kLoopsBoost : kLoopsQuiet);
        EXPECT_EQ(flight[i].p.since_ms, want[i].since);
        EXPECT_TRUE(boardTakes(flight[i].p.h));
    }
    // The last pad frame before it was a pad (the cue's boost is a change)
    const auto before = std::find_if(out.rbegin(), out.rend(), [&](const Frame& fr) { return fr.now < T; });
    ASSERT_NE(before, out.rend());
    EXPECT_EQ(before->p.phase, kPhPad);
}

// ── The flight image's attitude and rests (projects/flight_computer/main/
//    sdr_aid_policy.h, main.cpp sdrAidNoteUpdate and sdrAidStep) ─────────

namespace {

using sdr_aid_fc::FilterState;
using sdr_aid_fc::RestDetector;
using sdr_aid_fc::RestWindow;
using sdr_aid_fc::kDegToRad;
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

// The aiding sender and the rest detector live at file scope in main.cpp,
// constant-initialized like the phase sender (these fail to compile
// otherwise).
[[maybe_unused]] constexpr sdr_aid_fc::Sender kFlightAidSender{};
[[maybe_unused]] constexpr RestDetector kFlightRest{};

// v turned by q (Hamilton, scalar first: q v q*), q normalized first.
Vec3 turn(const double q[4], const Vec3& v)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double k = 2.0 / (w * w + x * x + y * y + z * z);
    return {(1 - k * (y * y + z * z)) * v[0] + k * (x * y - w * z) * v[1] + k * (x * z + w * y) * v[2],
            k * (x * y + w * z) * v[0] + (1 - k * (x * x + z * z)) * v[1] + k * (y * z - w * x) * v[2],
            k * (x * z - w * y) * v[0] + k * (y * z + w * x) * v[1] + (1 - k * (x * x + y * y)) * v[2]};
}

void expectNear(const Vec3& got, const Vec3& want, double tol, const char* what)
{
    for (int k = 0; k < 3; k++) EXPECT_NEAR(got[k], want[k], tol) << what << " [" << k << "]";
}

// The local axes in ECEF at a geodetic lat, lon.
Vec3 northAt(double lat, double lon)
{
    return {-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)};
}
Vec3 eastAt(double, double lon) { return {-std::sin(lon), std::cos(lon), 0.0}; }
Vec3 downAt(double lat, double lon)
{
    return {-std::cos(lat) * std::cos(lon), -std::cos(lat) * std::sin(lon), -std::sin(lat)};
}

// WGS-84 geodetic to ECEF.
Vec3 ecefOf(double lat, double lon, double h)
{
    const double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2.0 - f);
    const double n = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    return {(n + h) * std::cos(lat) * std::cos(lon), (n + h) * std::cos(lat) * std::sin(lon),
            (n * (1.0 - e2) + h) * std::sin(lat)};
}

// Body to NED: level facing north, nose up (pitch +90 deg about the body's
// right axis), level facing east (yaw +90 deg about down).
const Quat kLevelNorth = {1.0, 0.0, 0.0, 0.0};
const Quat kNoseUp = {kR2, 0.0, kR2, 0.0};
const Quat kLevelEast = {kR2, 0.0, 0.0, kR2};

FilterState stateAt(int64_t t_us, const Quat& q, const Vec3& w, bool fix_place = false, double lat = 0.0,
                    double lon = 0.0)
{
    FilterState s;
    s.t_us = t_us;
    std::copy(q.begin(), q.end(), s.q_nb);
    std::copy(w.begin(), w.end(), s.w);
    s.fix_place = fix_place;
    s.lat = lat;
    s.lon = lon;
    return s;
}

// The header a stamp at MCU instant t_us takes off a line.
Header stampOn(const GnssClockData& line, int64_t t_us)
{
    double gps_s = 0.0;
    EXPECT_TRUE(lc86::gnssClockGpsAt(line, t_us, gps_s));
    return gpsStamp(gps_s);
}

void expectStamp(const Header& got, const Header& want)
{
    EXPECT_EQ(got.week, want.week);
    EXPECT_EQ(got.tow_ms, want.tow_ms);
    EXPECT_EQ(got.sub_ns, want.sub_ns);
}

// Samples to the detector every 2 ms (the filter's 500 Hz) over [t0, t1):
// one force and gyro, x alternately +-alt_x about it; the ends of the
// windows it said to send.
std::vector<int64_t> feedSpan(RestDetector& d, int64_t t0, int64_t t1, const Vec3& f, const Vec3& g,
                              double alt_x = 0.0)
{
    std::vector<int64_t> out;
    int k = 0;
    for (int64_t t = t0; t < t1; t += 2000, k++)
    {
        Vec3 fk = f;
        fk[0] += (k % 2 == 0) ? alt_x : -alt_x;
        if (d.feed(t, fk.data(), g.data())) out.push_back(d.window().t1_us);
    }
    return out;
}

const Vec3 kStillF = {0.0123, -0.4567, -9.7801};   // 9.7908 m/s^2: within 0.5 of 1 g
const Vec3 kBiasG = {0.4, -0.2, 0.1};              // a raw gyro's bias, deg/s

}  // namespace

TEST(SdrAidFc, NedToEcefTakesNorthEastAndDownToTheirEcefAxes)
{
    for (const auto& [lat_d, lon_d] : {std::pair<double, double>{0.0, 0.0}, {40.5, -112.1}, {-33.9, 151.2},
                                       {89.0, 10.0}, {-60.0, -179.9}})
    {
        SCOPED_TRACE(testing::Message() << lat_d << ", " << lon_d);
        const double lat = lat_d * kDegToRad, lon = lon_d * kDegToRad;
        double q[4];
        sdr_aid_fc::nedToEcef(lat, lon, q);
        EXPECT_NEAR(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1.0, 1e-15);
        expectNear(turn(q, {1, 0, 0}), northAt(lat, lon), 1e-12, "north");
        expectNear(turn(q, {0, 1, 0}), eastAt(lat, lon), 1e-12, "east");
        expectNear(turn(q, {0, 0, 1}), downAt(lat, lon), 1e-12, "down");
    }
}

TEST(SdrAidFc, ANoseUpBodyPointsUpAndALevelOneNorth)
{
    const double lat = 40.5 * kDegToRad, lon = -112.1 * kDegToRad;
    const Vec3 up = {-downAt(lat, lon)[0], -downAt(lat, lon)[1], -downAt(lat, lon)[2]};
    double q[4];
    // Level, facing north: forward along north, right along east, down down
    sdr_aid_fc::bodyToEcef(kLevelNorth.data(), lat, lon, q);
    expectNear(turn(q, {1, 0, 0}), northAt(lat, lon), 1e-12, "level: forward");
    expectNear(turn(q, {0, 1, 0}), eastAt(lat, lon), 1e-12, "level: right");
    expectNear(turn(q, {0, 0, 1}), downAt(lat, lon), 1e-12, "level: down");
    // Nose up on the pad: forward along the local up, its belly (down)
    // facing north, right still east
    sdr_aid_fc::bodyToEcef(kNoseUp.data(), lat, lon, q);
    expectNear(turn(q, {1, 0, 0}), up, 1e-12, "nose up: forward");
    expectNear(turn(q, {0, 0, 1}), northAt(lat, lon), 1e-12, "nose up: down");
    expectNear(turn(q, {0, 1, 0}), eastAt(lat, lon), 1e-12, "nose up: right");
    // Level, facing east
    sdr_aid_fc::bodyToEcef(kLevelEast.data(), lat, lon, q);
    expectNear(turn(q, {1, 0, 0}), eastAt(lat, lon), 1e-12, "east: forward");

    // Through the sender and the wire: the board's decoded quaternion (2^-14
    // steps) puts the nose along the local up
    sdr_aid_fc::Sender tx;
    const Vec3 pad = ecefOf(lat, lon, 1400.0);
    ASSERT_TRUE(tx.noteBoardFix(pad.data()));
    tx.noteFilter(stateAt(1000000, kNoseUp, {0.0, 0.0, 0.0}));
    int64_t stamp_us = 0;
    Attitude a = tx.attitude(1000500, false, kLineTq, stamp_us);
    ASSERT_TRUE(a.att_valid);
    uint8_t b[kMaxFrame];
    const size_t len = encode(a, b, sizeof(b));
    Attitude d;
    ASSERT_TRUE(decode(b, len, d));
    ASSERT_TRUE(d.att_valid);
    EXPECT_EQ(d.frame, kFrameImu);   // the IMU's axes: the body's, with no mounting
    expectNear(turn(d.q, {1, 0, 0}), up, 2e-4, "nose up off the wire");
}

TEST(SdrAidFc, TheAttitudeAndItsRateInThisImusAxes)
{
    // This IMU mounted with its +z toward the nose: body x = imu z, body y =
    // imu y, body z = -imu x (v_body = b2r v_imu). Nose up on the pad, the
    // IMU's z points up and its x north (the body's down, negated: the belly
    // faces north), and a body rate (0.1, 0.2, 0.3) is (-0.3, 0.2, 0.1) in
    // the IMU's axes
    const double lat = 40.5 * kDegToRad, lon = -112.1 * kDegToRad;
    const Vec3 up = {-downAt(lat, lon)[0], -downAt(lat, lon)[1], -downAt(lat, lon)[2]};
    sdr_aid_fc::Sender tx;
    const Vec3 pad = ecefOf(lat, lon, 1400.0);
    ASSERT_TRUE(tx.noteBoardFix(pad.data()));
    FilterState s = stateAt(1000000, kNoseUp, {0.1, 0.2, 0.3});
    const double b2r[9] = {0, 0, 1, 0, 1, 0, -1, 0, 0};
    std::copy(b2r, b2r + 9, s.b2r);
    tx.noteFilter(s);
    int64_t stamp_us = 0;
    const Attitude a = tx.attitude(1000500, false, kLineTq, stamp_us);
    ASSERT_TRUE(a.att_valid);
    ASSERT_TRUE(a.rate_valid);
    EXPECT_EQ(a.frame, kFrameImu);
    expectNear(turn(a.q, {0, 0, 1}), up, 1e-12, "the IMU's z");
    const Vec3 north = northAt(lat, lon);
    expectNear(turn(a.q, {1, 0, 0}), {-north[0], -north[1], -north[2]}, 1e-12, "the IMU's x");
    expectNear(turn(a.q, {0, 1, 0}), eastAt(lat, lon), 1e-12, "the IMU's y");
    EXPECT_NEAR(a.w[0], -0.3, 1e-15);
    EXPECT_NEAR(a.w[1], 0.2, 1e-15);
    EXPECT_NEAR(a.w[2], 0.1, 1e-15);
    EXPECT_GE(a.q[0], 0.0);
}

TEST(SdrAidFc, AMatrixsQuaternionFromEachOfShepperdsBranches)
{
    // A turn about each axis past 90 deg and the identity: each of the four
    // branches, back to the same matrix, scalar part >= 0
    const double c = std::cos(2.5), s = std::sin(2.5);
    const double mats[][9] = {
        {1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 0, 0, 0, c, -s, 0, s, c},
        {c, 0, s, 0, 1, 0, -s, 0, c},
        {c, -s, 0, s, c, 0, 0, 0, 1},
        {0, 0, 1, 0, 1, 0, -1, 0, 0},
    };
    for (const auto& R : mats)
    {
        double q[4];
        sdr_aid_fc::quatOfDcm(R, q);
        EXPECT_GE(q[0], 0.0);
        EXPECT_NEAR(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1.0, 1e-12);
        for (int j = 0; j < 3; j++)
        {
            Vec3 e = {0, 0, 0};
            e[j] = 1.0;
            const Vec3 col = turn(q, e);
            for (int i = 0; i < 3; i++) EXPECT_NEAR(col[i], R[3 * i + j], 1e-12) << i << j;
        }
        // and imuOf undoes R
        const double v[3] = {0.3, -1.2, 9.7};
        double body[3], back[3];
        for (int i = 0; i < 3; i++) body[i] = R[3 * i] * v[0] + R[3 * i + 1] * v[1] + R[3 * i + 2] * v[2];
        sdr_aid_fc::imuOf(R, body, back);
        for (int i = 0; i < 3; i++) EXPECT_NEAR(back[i], v[i], 1e-12);
    }
}

TEST(SdrAidFc, TheBoardsOwnFixMakesItsRoundTripExact)
{
    // The board takes body to NED back as C_ne(its fix) C_eb: with the place
    // its own fix, that is the filter's q_nb whatever the place
    const double lat = -33.9 * kDegToRad, lon = 151.2 * kDegToRad;
    const Vec3 fix = ecefOf(lat, lon, 35.0);
    sdr_aid_fc::Sender tx;
    ASSERT_TRUE(tx.noteBoardFix(fix.data()));
    const Quat q_nb = {0.5, -0.5, 0.5, 0.5};
    tx.noteFilter(stateAt(5000000, q_nb, {0.0, 0.0, 0.0}));
    int64_t stamp_us = 0;
    const Attitude a = tx.attitude(5000000, false, kLineTq, stamp_us);
    ASSERT_TRUE(a.att_valid);
    double q_en[4];
    sdr_aid_fc::nedToEcef(lat, lon, q_en);
    const double q_ne[4] = {q_en[0], -q_en[1], -q_en[2], -q_en[3]};
    double back[4];
    sdr_aid_fc::quatMul(q_ne, a.q, back);
    for (int k = 0; k < 4; k++) EXPECT_NEAR(back[k], q_nb[k], 1e-9) << k;   // Bowring's latitude, 1e-12 rad here
}

TEST(SdrAidFc, LatLonOfAnEcefPosition)
{
    for (const double lat_d : {0.0, 39.97, -45.0, 89.9, -89.9})
        for (const double lon_d : {0.0, -74.94, 179.99, -119.0})
            for (const double h : {-100.0, 0.0, 1500.0, 12000.0})
            {
                const double lat = lat_d * kDegToRad, lon = lon_d * kDegToRad;
                const Vec3 p = ecefOf(lat, lon, h);
                double got_lat = 0.0, got_lon = 0.0;
                ASSERT_TRUE(sdr_aid_fc::latLonOf(p.data(), got_lat, got_lon));
                EXPECT_NEAR(got_lat, lat, 1e-9) << lat_d << " " << lon_d << " " << h;
                EXPECT_NEAR(std::remainder(got_lon - lon, 2 * M_PI), 0.0, 1e-9) << lat_d << " " << lon_d << " " << h;
            }
    // The board's fix in the recorded stream (test_sdr_rtcm.cpp TheFix):
    // the bench antenna
    const double fix[3] = {1272103.5908, -4726528.5163, 4075563.8196};
    double lat = 0.0, lon = 0.0;
    ASSERT_TRUE(sdr_aid_fc::latLonOf(fix, lat, lon));
    EXPECT_NEAR(lat / kDegToRad, 39.971621, 1e-6);
    EXPECT_NEAR(lon / kDegToRad, -74.936297, 1e-6);
    // Nothing off the Earth: no position, not a number, too deep or too high
    const double zero[3] = {0.0, 0.0, 0.0};
    const double nan3[3] = {kNaN, 0.0, 6.4e6};
    const Vec3 deep = ecefOf(0.0, 0.0, -400000.0);
    const Vec3 high = ecefOf(0.0, 0.0, 450000.0);
    EXPECT_FALSE(sdr_aid_fc::latLonOf(zero, lat, lon));
    EXPECT_FALSE(sdr_aid_fc::latLonOf(nan3, lat, lon));
    EXPECT_FALSE(sdr_aid_fc::latLonOf(deep.data(), lat, lon));
    EXPECT_FALSE(sdr_aid_fc::latLonOf(high.data(), lat, lon));
}

TEST(SdrAidFc, AThirtyTwoBitTimeWidenedNearNow)
{
    const int64_t wrap = int64_t{1} << 32;
    EXPECT_EQ(sdr_aid_fc::widenUs(7000, 10000), 7000);
    EXPECT_EQ(sdr_aid_fc::widenUs(static_cast<uint32_t>(wrap + 5000 - 3000), wrap + 5000), wrap + 2000);
    // Across the 32-bit wrap: just before it, read just after it
    EXPECT_EQ(sdr_aid_fc::widenUs(0xFFFFFF38u, wrap + 100), wrap - 200);
    // A little after now (the IMU's stamp ahead of the pass): still that
    EXPECT_EQ(sdr_aid_fc::widenUs(static_cast<uint32_t>(3 * wrap + 10), 3 * wrap - 5), 3 * wrap + 10);
}

TEST(RestDetector, BackToBackWindowsOfASecond)
{
    RestDetector d;
    const int64_t t0 = 10000000;
    const std::vector<int64_t> sent = feedSpan(d, t0, t0 + 3500000, kStillF, kBiasG);
    EXPECT_EQ(d.windows(), 3u);
    EXPECT_EQ(sent, (std::vector<int64_t>{t0 + 1000000}));   // the first; the next 5 s on
    // The last: from the end of the one before to its last sample, 500
    // samples, the mean as fed, no spread, the gyro's bias
    const RestWindow& w = d.window();
    EXPECT_EQ(w.t0_us, t0 + 2000000);
    EXPECT_EQ(w.t1_us, t0 + 3000000);
    EXPECT_EQ(w.n, 500u);
    for (int k = 0; k < 3; k++) EXPECT_NEAR(w.f[k], kStillF[k], 1e-12) << k;
    EXPECT_NEAR(w.sd_f, 0.0, 1e-9);
    EXPECT_NEAR(w.gyro_max_dps, std::sqrt(0.16 + 0.04 + 0.01), 1e-12);
    EXPECT_TRUE(w.still);
    EXPECT_EQ(d.run(), 3u);
    // As the board reads it: 1000 ms, stamped at its middle (sdr_aid_fc::Sender)
    const Rest r = sdr_aid_fc::restOf(w);
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.dur_ms, 1000);
    EXPECT_EQ(r.f[2], w.f[2]);
    EXPECT_EQ(r.sd_f, w.sd_f);
    // The spread: x +-a about its mean on alternate samples is a / sqrt(3) a
    // axis
    RestDetector e;
    feedSpan(e, t0, t0 + 1000001, kStillF, kBiasG, 0.03);
    ASSERT_EQ(e.windows(), 1u);
    EXPECT_NEAR(e.window().sd_f, 0.03 / std::sqrt(3.0), 1e-9);
    EXPECT_NEAR(e.window().f[0], kStillF[0], 1e-12);
}

TEST(RestDetector, TheStillnessTest)
{
    // 1 g within 0.5 m/s^2, a spread under 0.05 m/s^2 a axis, every |gyro|
    // under 3 deg/s; each window alone
    struct Row { Vec3 f; Vec3 g; double alt_x; bool still; const char* what; };
    const double g0 = RestDetector::kG;
    const Row rows[] = {
        {{0.0, 0.0, -(g0 + 0.49)}, kBiasG, 0.0, true, "1 g + 0.49"},
        {{0.0, 0.0, -(g0 + 0.51)}, kBiasG, 0.0, false, "1 g + 0.51"},
        {{0.0, 0.0, -(g0 - 0.49)}, kBiasG, 0.0, true, "1 g - 0.49"},
        {{0.0, 0.0, -(g0 - 0.51)}, kBiasG, 0.0, false, "1 g - 0.51"},
        {{g0, 0.0, 0.0}, kBiasG, 0.0, true, "nose up (1 g along +x)"},
        {kStillF, kBiasG, 0.08, true, "a spread of 0.046"},
        {kStillF, kBiasG, 0.09, false, "a spread of 0.052"},
        {kStillF, {2.99, 0.0, 0.0}, 0.0, true, "2.99 deg/s"},
        {kStillF, {0.0, 0.0, -3.01}, 0.0, false, "3.01 deg/s"},
        {kStillF, {1.8, 1.8, 1.8}, 0.0, false, "3.12 deg/s, under 3 on each axis"},
        {{kNaN, 0.0, -g0}, kBiasG, 0.0, false, "a NaN force"},
        {kStillF, {kNaN, 0.0, 0.0}, 0.0, false, "a NaN rate"},
    };
    for (const Row& r : rows)
    {
        SCOPED_TRACE(r.what);
        RestDetector d;
        const std::vector<int64_t> sent = feedSpan(d, 0, 1000001, r.f, r.g, r.alt_x);
        ASSERT_EQ(d.windows(), 1u);
        EXPECT_EQ(d.window().still, r.still);
        EXPECT_EQ(sent.size(), r.still ? 1u : 0u);   // nothing goes for a window not still
        EXPECT_EQ(d.run(), r.still ? 1u : 0u);
    }
}

TEST(RestDetector, TheFirstThenEveryFifthThenANewPoseAtOnce)
{
    RestDetector d;
    const int64_t s = 1000000;
    // 30 s still: windows 1, 6, 11, 16, 21, 26
    std::vector<int64_t> sent = feedSpan(d, 0, 30 * s + 1, kStillF, kBiasG);
    EXPECT_EQ(sent, (std::vector<int64_t>{1 * s, 6 * s, 11 * s, 16 * s, 21 * s, 26 * s}));
    // 1.5 s of a move (20 deg/s): two windows not still, nothing
    sent = feedSpan(d, 30 * s + 2000, 31500000, kStillF, {20.0, 0.0, 0.0});
    EXPECT_TRUE(sent.empty());
    EXPECT_EQ(d.run(), 0u);
    // Still again, nose up now: the first whole still window goes at once
    // (32-33 s), then every fifth again
    sent = feedSpan(d, 31500000, 45 * s + 1, {9.80, 0.01, -0.02}, kBiasG);
    EXPECT_EQ(sent, (std::vector<int64_t>{33 * s, 38 * s, 43 * s}));
    EXPECT_NEAR(d.window().f[0], 9.80, 1e-12);
    // Never still: nothing at all
    RestDetector m;
    EXPECT_TRUE(feedSpan(m, 0, 10 * s + 1, kStillF, {0.0, 5.0, 0.0}).empty());
    EXPECT_EQ(m.windows(), 10u);
}

TEST(RestDetector, AGapInTheSamplesBreaksTheRun)
{
    // A stopped filter (MAG_CALIBRATION, a reinit) could hide a move: a gap
    // over 100 ms throws the open window away and ends the run, so the first
    // whole window after it goes at once, from the first sample after it
    RestDetector d;
    const int64_t s = 1000000;
    std::vector<int64_t> sent = feedSpan(d, 0, 3500001, kStillF, kBiasG);
    EXPECT_EQ(sent, (std::vector<int64_t>{1 * s}));
    sent = feedSpan(d, 3700000, 10 * s, kStillF, kBiasG);   // 3.5 to 3.7 s: no samples
    EXPECT_EQ(sent, (std::vector<int64_t>{4700000, 9700000}));
    EXPECT_EQ(d.windows(), 3u + 6u);
    // A gap of 100 ms or less is no break: the window spans it
    RestDetector e;
    feedSpan(e, 0, 400001, kStillF, kBiasG);
    feedSpan(e, 500000, 1100001, kStillF, kBiasG);
    ASSERT_EQ(e.windows(), 1u);
    EXPECT_EQ(e.window().t0_us, 0);
    EXPECT_EQ(e.window().t1_us, 1000000);
    EXPECT_EQ(e.window().n, 200u + 251u);   // 2-400 ms, then 500-1000 ms
    // A sample not after the last is passed over
    RestDetector f;
    const double fz[3] = {0.0, 0.0, -9.8}, gz[3] = {0.0, 0.0, 0.0};
    EXPECT_FALSE(f.feed(1000, fz, gz));
    EXPECT_FALSE(f.feed(1000, fz, gz));
    EXPECT_FALSE(f.feed(900, fz, gz));
    EXPECT_EQ(f.windows(), 0u);
}

TEST(SdrAidFc, TheAttitudeTenTimesASecondOnceTheLineIsValid)
{
    const GnssClockData line = validLine();
    sdr_phase::Sender ph;
    sdr_aid_fc::Sender tx;
    const double lat = 40.5 * kDegToRad, lon = -112.1 * kDegToRad;
    const Vec3 up = {-downAt(lat, lon)[0], -downAt(lat, lon)[1], -downAt(lat, lon)[2]};
    const Vec3 pad = ecefOf(lat, lon, 1400.0);
    ASSERT_TRUE(tx.noteBoardFix(pad.data()));
    const Vec3 w = {0.01, -0.02, 0.5};
    const int64_t t0 = 20000000;
    struct Got { int64_t now; Attitude a; };
    std::vector<Got> got;
    for (int64_t now = t0; now < t0 + 3000000; now += 1000)
    {
        // The line valid from 1 s in, noted once a second; the filter every
        // 2 ms, its IMU sample 0.7 ms before the pass
        if (now >= t0 + 1000000 && (now - t0) % 1000000 == 0) ph.noteLine(line, now);
        if ((now - t0) % 2000 == 0) tx.noteFilter(stateAt(now - 700, kNoseUp, w));
        uint8_t b[kMaxFrame];
        const size_t len = tx.frame(now, ph, false, b, sizeof(b));
        if (len == 0) continue;
        ASSERT_EQ(len, kAttitudeFrame);
        Attitude a;
        ASSERT_TRUE(decode(b, len, a));
        got.push_back({now, a});
        tx.sent(now);
    }
    ASSERT_EQ(got.size(), 20u);
    for (size_t i = 0; i < got.size(); i++)
    {
        SCOPED_TRACE(testing::Message() << "frame " << i);
        const Attitude& a = got[i].a;
        EXPECT_EQ(got[i].now, t0 + 1000000 + static_cast<int64_t>(i) * 100000);
        expectStamp(a.h, stampOn(line, got[i].now - 700));   // the filter state's instant
        EXPECT_EQ(a.h.seq, i);
        EXPECT_EQ(a.h.src, kSrcOwn);
        EXPECT_FALSE(a.h.test);
        EXPECT_EQ(a.h.tq, kLineTq);
        EXPECT_TRUE(boardTakes(a.h));
        EXPECT_TRUE(a.att_valid);
        EXPECT_TRUE(a.rate_valid);
        EXPECT_EQ(a.frame, kFrameImu);   // this IMU's axes (the body's: no mounting here)
        EXPECT_TRUE(std::isnan(a.sig_tilt_deg));
        EXPECT_TRUE(std::isnan(a.sig_hdg_deg));
        expectNear(turn(a.q, {1, 0, 0}), up, 2e-4, "the nose");
        for (int k = 0; k < 3; k++) EXPECT_EQ(lsb(a.w[k], 1024.0), lsb(w[k], 1024.0)) << k;
    }
    EXPECT_EQ(tx.attSeq(), 20);
}

TEST(SdrAidFc, NotValidWithoutAFreshFilterOrAPlace)
{
    sdr_aid_fc::Sender tx;
    int64_t stamp_us = 0;
    const int64_t t = 50000000;
    // No filter state yet (before its start): neither valid, stamped at the
    // pass
    Attitude a = tx.attitude(t, false, kLineTq, stamp_us);
    EXPECT_FALSE(a.att_valid);
    EXPECT_FALSE(a.rate_valid);
    EXPECT_EQ(stamp_us, t);
    // A state, no place (the GNSS-absent start's 0 N 0 E is none, and no fix
    // from the board yet): its rate alone
    tx.noteFilter(stateAt(t - 1000, kLevelNorth, {0.1, 0.2, 0.3}, false, 0.0, 0.0));
    a = tx.attitude(t, false, kLineTq, stamp_us);
    EXPECT_FALSE(a.att_valid);
    EXPECT_TRUE(a.rate_valid);
    EXPECT_EQ(stamp_us, t - 1000);
    // A filter on a fix of its own: its place
    const double lat = 33.7 * kDegToRad, lon = -118.4 * kDegToRad;
    tx.noteFilter(stateAt(t - 1000, kLevelNorth, {0.1, 0.2, 0.3}, true, lat, lon));
    a = tx.attitude(t, false, kLineTq, stamp_us);
    ASSERT_TRUE(a.att_valid);
    expectNear(turn(a.q, {1, 0, 0}), northAt(lat, lon), 1e-12, "the filter's place");
    // The board's fix, once there is one, before the filter's
    const double blat = 39.97 * kDegToRad, blon = -74.94 * kDegToRad;
    double got_lat = 0.0, got_lon = 0.0;
    EXPECT_FALSE(tx.boardPlace(got_lat, got_lon));
    ASSERT_TRUE(tx.noteBoardFix(ecefOf(blat, blon, -11.5).data()));
    EXPECT_TRUE(tx.haveBoardPlace());
    ASSERT_TRUE(tx.boardPlace(got_lat, got_lon));
    EXPECT_NEAR(got_lat, blat, 1e-9);
    EXPECT_NEAR(got_lon, blon, 1e-9);
    a = tx.attitude(t, false, kLineTq, stamp_us);
    ASSERT_TRUE(a.att_valid);
    expectNear(turn(a.q, {1, 0, 0}), northAt(blat, blon), 1e-9, "the board's place");
    // ... and a position off the Earth changes nothing
    const double zero[3] = {0.0, 0.0, 0.0};
    EXPECT_FALSE(tx.noteBoardFix(zero));
    a = tx.attitude(t, false, kLineTq, stamp_us);
    expectNear(turn(a.q, {1, 0, 0}), northAt(blat, blon), 1e-9, "still the board's place");
    // A NaN: its group alone not valid
    tx.noteFilter(stateAt(t - 1000, kLevelNorth, {kNaN, 0.2, 0.3}));
    a = tx.attitude(t, false, kLineTq, stamp_us);
    EXPECT_TRUE(a.att_valid);
    EXPECT_FALSE(a.rate_valid);
    tx.noteFilter(stateAt(t - 1000, {kNaN, 0.0, 0.0, 1.0}, {0.1, 0.2, 0.3}));
    a = tx.attitude(t, false, kLineTq, stamp_us);
    EXPECT_FALSE(a.att_valid);
    EXPECT_TRUE(a.rate_valid);
    // The filter stopped (no update for over 100 ms): not valid, stamped at
    // the pass, so the board drops what it holds at once
    tx.noteFilter(stateAt(t, kLevelNorth, {0.1, 0.2, 0.3}));
    a = tx.attitude(t + sdr_aid_fc::Sender::kAttFreshUs, false, kLineTq, stamp_us);
    EXPECT_TRUE(a.att_valid);
    EXPECT_EQ(stamp_us, t);
    a = tx.attitude(t + sdr_aid_fc::Sender::kAttFreshUs + 1, false, kLineTq, stamp_us);
    EXPECT_FALSE(a.att_valid);
    EXPECT_FALSE(a.rate_valid);
    EXPECT_EQ(stamp_us, t + sdr_aid_fc::Sender::kAttFreshUs + 1);
}

TEST(SdrAidFc, ARestGoesAfterTheAttitudeStampedAtItsWindowsMiddle)
{
    const GnssClockData line = validLine();
    sdr_phase::Sender ph;
    sdr_aid_fc::Sender tx;
    const int64_t T = 80000000;
    ph.noteLine(line, T);
    tx.noteFilter(stateAt(T, kNoseUp, {0.0, 0.0, 0.0}));
    RestWindow w;
    w.t0_us = T - 1000000;
    w.t1_us = T;
    w.n = 500;
    std::copy(kStillF.begin(), kStillF.end(), w.f);
    w.sd_f = 0.0042;
    w.still = true;
    tx.noteRest(w);
    EXPECT_TRUE(tx.restPending());
    uint8_t b[kMaxFrame];
    // The attitude due on the same pass goes first
    size_t len = tx.frame(T + 500, ph, false, b, sizeof(b));
    ASSERT_EQ(len, kAttitudeFrame);
    tx.sent(T + 500);
    // The rest on the next: R1's values, stamped at the window's middle
    len = tx.frame(T + 1500, ph, false, b, sizeof(b));
    ASSERT_EQ(len, kRestFrame);
    Rest r;
    ASSERT_TRUE(decode(b, len, r));
    expectStamp(r.h, stampOn(line, T - 500000));
    EXPECT_EQ(r.h.seq, 0);
    EXPECT_EQ(r.h.src, kSrcOwn);
    EXPECT_FALSE(r.h.test);
    EXPECT_EQ(r.h.tq, kLineTq);
    EXPECT_TRUE(boardTakes(r.h));
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.frame, kFrameImu);   // the detector was fed this IMU's axes
    EXPECT_EQ(r.dur_ms, 1000);
    EXPECT_EQ(lsb(r.f[0], 10000.0), 123);
    EXPECT_EQ(lsb(r.f[1], 10000.0), -4567);
    EXPECT_EQ(lsb(r.f[2], 10000.0), -97801);
    EXPECT_EQ(code(r.sd_f, 1000.0, 255), 5);
    {
        // R1's values in this IMU's axes: R5, byte for byte, but for the stamp
        Rest want = kInRest1;
        want.frame = kFrameImu;
        want.h = r.h;
        EXPECT_EQ(Bytes(b, b + len), enc(want));
    }
    // No room in the UART (sent() not called): the same frame on the next
    // pass, then gone once it went
    const Bytes first(b, b + len);
    len = tx.frame(T + 2500, ph, false, b, sizeof(b));
    EXPECT_EQ(Bytes(b, b + len), first);
    tx.sent(T + 2500);
    EXPECT_FALSE(tx.restPending());
    EXPECT_EQ(tx.restSeq(), 1);
    EXPECT_EQ(tx.frame(T + 3500, ph, false, b, sizeof(b)), 0u);
    // A sim's or a rig replay's: a test frame, which the board takes only
    // under its `rx fc test on`
    w.t0_us += 5000000;
    w.t1_us += 5000000;
    tx.noteRest(w);
    len = tx.frame(T + 5000500, ph, true, b, sizeof(b));   // the attitude, due again: the filter long stopped
    ASSERT_EQ(len, kAttitudeFrame);
    Attitude a;
    ASSERT_TRUE(decode(b, len, a));
    EXPECT_TRUE(a.h.test);
    EXPECT_FALSE(a.att_valid);
    expectStamp(a.h, stampOn(line, T + 5000500));
    tx.sent(T + 5000500);
    len = tx.frame(T + 5001500, ph, true, b, sizeof(b));
    ASSERT_EQ(len, kRestFrame);
    ASSERT_TRUE(decode(b, len, r));
    EXPECT_TRUE(r.h.test);
    EXPECT_EQ(r.h.seq, 1);
    EXPECT_FALSE(boardTakes(r.h));
    tx.sent(T + 5001500);
}

TEST(SdrAidFc, ARestWithNoLineGoesAtOnceUnstamped)
{
    // Indoors, before the board's first fix: no line. The rest goes on the
    // first pass after its window, tq 0 and no stamp (the board places it at
    // its arrival less half its length), in this IMU's axes; the attitude
    // does not go
    sdr_phase::Sender ph;   // no line
    sdr_aid_fc::Sender tx;
    tx.noteFilter(stateAt(10000000, kNoseUp, {0.0, 0.0, 0.0}));
    RestWindow w;
    w.t0_us = 9000000;
    w.t1_us = 10000000;
    w.n = 500;
    std::copy(kStillF.begin(), kStillF.end(), w.f);
    w.sd_f = 0.0042;
    w.still = true;
    tx.noteRest(w);
    uint8_t b[kMaxFrame];
    size_t len = tx.frame(10000500, ph, false, b, sizeof(b));
    ASSERT_EQ(len, kRestFrame);
    Rest r;
    ASSERT_TRUE(decode(b, len, r));
    EXPECT_EQ(r.h.week, 0);
    EXPECT_EQ(r.h.tow_ms, 0u);
    EXPECT_EQ(r.h.sub_ns, 0);
    EXPECT_EQ(r.h.tq, 0);
    EXPECT_EQ(r.h.seq, 0);
    EXPECT_EQ(r.h.src, kSrcOwn);
    EXPECT_FALSE(r.h.test);
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.frame, kFrameImu);
    EXPECT_EQ(r.dur_ms, 1000);
    EXPECT_EQ(lsb(r.f[2], 10000.0), -97801);
    tx.sent(10000500);
    EXPECT_FALSE(tx.restPending());
    EXPECT_EQ(tx.restSeq(), 1);
    // Nothing more: no attitude without a line
    for (int64_t now = 10001500; now < 11000000; now += 1000)
        ASSERT_EQ(tx.frame(now, ph, false, b, sizeof(b)), 0u);

    // One that found no room until past 100 ms after its window: dropped (its
    // arrival would misplace it)
    w.t0_us = 20000000;
    w.t1_us = 21000000;
    tx.noteRest(w);
    EXPECT_EQ(tx.frame(21000000 + sdr_aid_fc::Sender::kRestNoLineUs + 1, ph, false, b, sizeof(b)), 0u);
    EXPECT_FALSE(tx.restPending());
    EXPECT_EQ(tx.restsDropped(), 1u);
    // At 100 ms it still goes
    w.t0_us = 30000000;
    w.t1_us = 31000000;
    tx.noteRest(w);
    ASSERT_EQ(tx.frame(31000000 + sdr_aid_fc::Sender::kRestNoLineUs, ph, false, b, sizeof(b)), kRestFrame);
    tx.sent(31100000);

    // With the line, stamped as before, and a stamped one still waits up to
    // 3 s for room
    const GnssClockData line = validLine();
    ph.noteLine(line, 40000000);
    w.t0_us = 39000000;
    w.t1_us = 40000000;
    tx.noteRest(w);
    len = tx.frame(40000000, ph, false, b, sizeof(b));   // the attitude first
    ASSERT_EQ(len, kAttitudeFrame);
    tx.sent(40000000);
    len = tx.frame(40001000, ph, false, b, sizeof(b));
    ASSERT_EQ(len, kRestFrame);
    ASSERT_TRUE(decode(b, len, r));
    expectStamp(r.h, stampOn(line, 39500000));
    EXPECT_EQ(r.h.tq, kLineTq);
    EXPECT_EQ(r.h.seq, 2);
    tx.sent(40001000);
    EXPECT_EQ(tx.restsDropped(), 1u);
}

TEST(SdrAidFc, APadAMoveAndANewPose)
{
    // main.cpp's order pass by pass, 1 ms apart: each filter update (every
    // 2 ms) feeds the detector and the sender, then the pass sends what is
    // due. 20 s still nose up, a 1.5 s move, still again to 30 s. The board's
    // line valid from 5 s, noted once a second; its fix from 2 s.
    const GnssClockData line = validLine();
    sdr_phase::Sender ph;
    sdr_aid_fc::Sender tx;
    RestDetector det;
    const double lat = 40.5 * kDegToRad, lon = -112.1 * kDegToRad;
    const Vec3 pad = ecefOf(lat, lon, 1400.0);
    const int64_t t0 = 100000000, s = 1000000;
    const Vec3 nose_up_f = {9.79, 0.012, -0.034};
    std::vector<int64_t> att_at;
    struct GotRest { int64_t now; Rest r; };
    std::vector<GotRest> rests;
    for (int64_t now = t0; now < t0 + 30 * s; now += 1000)
    {
        const int64_t rel = now - t0;
        if (rel >= 5 * s && rel % s == 0) ph.noteLine(line, now);
        if (rel == 2 * s) ASSERT_TRUE(tx.noteBoardFix(pad.data()));
        if (rel % 2000 == 0)
        {
            const bool moving = rel >= 20 * s && rel < 21500000;
            const Vec3 g = moving ? Vec3{0.0, 25.0, 0.0} : kBiasG;
            if (det.feed(now, nose_up_f.data(), g.data())) tx.noteRest(det.window());
            tx.noteFilter(stateAt(now, kNoseUp, {g[0] * kDegToRad, g[1] * kDegToRad, g[2] * kDegToRad}));
        }
        uint8_t b[kMaxFrame];
        const size_t len = tx.frame(now, ph, false, b, sizeof(b));
        if (len == 0) continue;
        if (fcSubType(b, len) == kAttitude)
        {
            att_at.push_back(rel);
        }
        else
        {
            ASSERT_EQ(fcSubType(b, len), kRest);
            Rest r;
            ASSERT_TRUE(decode(b, len, r));
            rests.push_back({rel, r});
        }
        tx.sent(now);
    }
    // The attitude every 100 ms from the line's first pass
    ASSERT_EQ(att_at.size(), 250u);
    for (size_t i = 0; i < att_at.size(); i++) EXPECT_EQ(att_at[i], 5 * s + static_cast<int64_t>(i) * 100000) << i;
    // The rests: the first window's (0-1 s) goes as it closes, with no line
    // yet: unstamped (tq 0), for the board to place at its arrival; the
    // sixth's (5-6 s) goes as it closes, the attitude ahead of it on that
    // pass, then every 5 s; after the move the first whole still window
    // (22-23 s) and the one five on
    ASSERT_FALSE(rests.empty());
    EXPECT_EQ(rests[0].now, s);
    EXPECT_EQ(rests[0].r.h.tq, 0);
    EXPECT_EQ(rests[0].r.h.week, 0);
    EXPECT_EQ(rests[0].r.h.tow_ms, 0u);
    EXPECT_EQ(rests[0].r.h.seq, 0);
    EXPECT_TRUE(rests[0].r.valid);
    rests.erase(rests.begin());
    const std::vector<int64_t> want_mid = {5500000, 10500000, 15500000, 22500000, 27500000};
    ASSERT_EQ(rests.size(), want_mid.size());
    for (size_t i = 0; i < rests.size(); i++)
    {
        SCOPED_TRACE(testing::Message() << "rest " << i);
        const Rest& r = rests[i].r;
        expectStamp(r.h, stampOn(line, t0 + want_mid[i]));
        // Each window closes on a whole second, with an attitude due: the
        // rest on the pass after
        EXPECT_EQ(rests[i].now, want_mid[i] + 500000 + 1000);
        EXPECT_EQ(r.h.seq, i + 1);
        EXPECT_TRUE(r.valid);
        EXPECT_EQ(r.dur_ms, 1000);
        for (int k = 0; k < 3; k++) EXPECT_EQ(lsb(r.f[k], 10000.0), lsb(nose_up_f[k], 10000.0)) << k;
        EXPECT_EQ(code(r.sd_f, 1000.0, 255), 1);   // no spread: the least code
    }
    EXPECT_EQ(tx.restsDropped(), 0u);
}

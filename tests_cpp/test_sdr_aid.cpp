// Host tests for TinkerRocket's message 4002, the flight computer to the
// gnss-sdr-dev board (TR_GNSSReceiverSdr_Serial/SdrAid.cpp): the golden
// frames both repos pin, the stamp, the P1 bench script and the schedule that
// paces it, and the aiding block the board appends to its 4001 epoch, with
// the frames it says it took. The golden bytes come from the P1 spec's
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
// E2: field ends: 40 marks asked (32 sent) and every block field at its top:
// the longest epoch, 566 bits
const Bytes kE2 = {
    0xD3, 0x00, 0x47, 0xFA, 0x11, 0xFF, 0xFF, 0x90, 0x32, 0x0F, 0xFD, 0xFF,
    0xFF, 0xC0, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x05, 0x42, 0xA0, 0xFC, 0x44, 0x16,
    0x46, 0xE1, 0xDC, 0x88, 0x27, 0x4A, 0x62, 0xEC, 0xCC, 0x35, 0x4E, 0xA3,
    0xFD, 0x04, 0x46, 0x52, 0xE4, 0xDD, 0x48, 0x57, 0x56, 0x65, 0xED, 0x8C,
    0x65, 0x5A, 0xA6, 0xFD, 0xC4, 0x76, 0x5E, 0xE7, 0xDE, 0x0B, 0xFF, 0xFF,
    0x83, 0xFC, 0x60, 0x0F, 0x52};
// E3: no marks, GPS aiding in the boost by the GPS channels' own decision
const Bytes kE3 = {
    0xD3, 0x00, 0x1F, 0xFA, 0x11, 0x09, 0x80, 0x30, 0xA3, 0x2D, 0xE0, 0x01,
    0x34, 0x80, 0x00, 0xC0, 0x00, 0x00, 0x01, 0x24, 0xC4, 0x15, 0xEA, 0xF3,
    0x7B, 0xC0, 0x48, 0xD1, 0x40, 0x00, 0x53, 0x00, 0x00, 0x00, 0xA7, 0xBC,
    0x81};

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

// Every golden 4002 frame, its sub-type with it.
std::vector<std::pair<uint8_t, Bytes>> allFc()
{
    return {{kAttitude, kA1}, {kAttitude, kA2}, {kAttitude, kA3}, {kAttitude, kA4}, {kNavState, kN1},
            {kNavState, kN2}, {kNavState, kN3}, {kNavState, kN4}, {kPhase, kP1}, {kPhase, kP2},
            {kPhase, kP3}, {kGeometry, kG1}, {kGeometry, kG2}, {kGeometry, kG3}, {kGeometry, kG4},
            {kAttitude, kS1_204000000}, {kAttitude, kS1_204005000}, {kAttitude, kS1_204017250},
            {kNavState, kS2_204000000}, {kNavState, kS2_204002500}, {kNavState, kS2_204055000},
            {kPhase, kS3_204000000}, {kPhase, kS3_204000160}, {kPhase, kS3_204010000},
            {kPhase, kS3_204080100}, {kPhase, kS3_204500000}, {kGeometry, kS4_204000000}};
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
        default: return reread<Geometry>(f);
    }
}

// Whether each of the four decoders takes a frame.
std::array<bool, 4> takenBy(const Bytes& f)
{
    Attitude a;
    NavState n;
    PhaseMsg p;
    Geometry g;
    return {decode(f.data(), f.size(), a), decode(f.data(), f.size(), n), decode(f.data(), f.size(), p),
            decode(f.data(), f.size(), g)};
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
    // Nothing known: the fields go out as 0 whatever the struct held
    checkAtt("A3", kA3, kInA3, false, false, {0, 0, 0, 0}, {0, 0, 0}, 0, 0);
    checkNav("N3", kN3, kInN3, true, false, {-30927639484, -55794938587, 0}, {0, 0, 0}, {300, 500, 0, 0});
    checkGeom("G3", kG3, kInG3, false, false, {0, 0, 0}, {0, 0, 0, 0});
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
    EXPECT_EQ(encode(kInA1, nullptr, sizeof(b)), 0u);
    EXPECT_EQ(encode(kInA1, b, kAttitudeFrame), kAttitudeFrame);
    EXPECT_EQ(encode(kInN1, b, kNavStateFrame), kNavStateFrame);
    EXPECT_EQ(encode(kInP1, b, kPhaseFrame), kPhaseFrame);
    EXPECT_EQ(encode(kInG1, b, kGeometryFrame), kGeometryFrame);
}

TEST(SdrAid, ShortFramesRefusedLongerOnesReadTheSame)
{
    for (const auto& [sub, f] : allFc())
    {
        // One byte short, re-framed: refused by every decoder
        Bytes p = payloadOf(f);
        p.pop_back();
        EXPECT_EQ(takenBy(frameOf(p)), (std::array<bool, 4>{false, false, false, false}));
        // Four bytes more (a later revision's), re-framed: read as before
        p = payloadOf(f);
        p.insert(p.end(), {0xA5, 0x5A, 0xFF, 0x00});
        const Bytes longer = frameOf(p);
        std::array<bool, 4> want = {false, false, false, false};
        want[sub - 1] = true;
        EXPECT_EQ(takenBy(longer), want);
        EXPECT_EQ(rereadAs(sub, longer), f);
    }
}

TEST(SdrAid, EachDecoderTakesOnlyItsOwn)
{
    for (const auto& [sub, f] : allFc())
    {
        std::array<bool, 4> want = {false, false, false, false};
        want[sub - 1] = true;
        EXPECT_EQ(takenBy(f), want);
        EXPECT_EQ(fcSubType(f.data(), f.size()), sub);
        EXPECT_EQ(sdr_rtcm::messageNumber(f.data(), f.size()), kFcMessage);
        EXPECT_EQ(sdr_rtcm::trSubType(f.data(), f.size()), 0);   // no 4001
        sdr_rtcm::TrEpoch e;
        EpochAid aid;
        EXPECT_FALSE(sdr_rtcm::decodeTrEpoch(f.data(), f.size(), e));
        EXPECT_FALSE(decodeEpochAid(f.data(), f.size(), aid));
        // The same bytes as a reserved sub-type (0, 5-15): nobody's
        for (uint8_t other : {0, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15})
        {
            Bytes p = payloadOf(f);
            p[1] = static_cast<uint8_t>((p[1] & 0xF0) | other);
            const Bytes g = frameOf(p);
            EXPECT_EQ(fcSubType(g.data(), g.size()), other);
            EXPECT_EQ(takenBy(g), (std::array<bool, 4>{false, false, false, false})) << int(sub) << " as " << int(other);
        }
    }
    // The board's 4001 epochs are no 4002
    for (const Bytes* e : {&kE0, &kE1, &kE2, &kE3})
    {
        EXPECT_EQ(fcSubType(e->data(), e->size()), 0);
        EXPECT_EQ(takenBy(*e), (std::array<bool, 4>{false, false, false, false}));
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

// One pass of the bench step (main.cpp sdrAidBenchStep): anything due, then
// the phase key at the frame's stamp (keyAt, worked out only then), then the
// sub-type with it, sent. 0: nothing went.
template <typename KeyAt>
uint8_t benchPass(AidSchedule& s, int64_t now, KeyAt keyAt, uint32_t& key)
{
    if (!s.anyDue(now)) return 0;
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
    for (int64_t now = from; now < to; now += 1000)
    {
        uint32_t k = 0;
        const uint8_t sub = benchPass(s, now, [](int64_t) { return 9u; }, k);
        if (sub != 0) out.push_back({now, sub, k});
    }
}

}  // namespace

TEST(AidSchedule, RatesSpacingAndChanges)
{
    // The bench step's loop: 10 s of passes 1 ms apart, each up to 0.5 ms
    // late, one stall of 5 ms; the phase key from the script at each frame's
    // stamp, the hotshot scene's launch at 1.1637 s and its burnout at
    // 5.2537 s (off the attitude's 10 ms grid).
    AidSchedule s;
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
        const uint8_t sub = benchPass(s, now, keyAt, k);
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

    // Each change, then its ten repeats 100 ms apart from 3 ms after it (off
    // the attitude's pass the change took), then one a second until the next
    // change: each at or after its time and within a few passes of it (a
    // frame due with it goes first, and a pass on time after one 0.5 ms late
    // is inside the 1 ms spacing: 4.2 ms at most over 120 000 runs of this
    // loop's jitter and alignments).
    int changes = 0;
    for (size_t i = 0; i < phase.size();)
    {
        changes++;
        const int64_t c = phase[i].t;
        size_t j = i + 1;
        for (int n = 1; j < phase.size() && phase[j].key == phase[i].key; j++, n++)
        {
            const int64_t due = c + 3000 + (n <= 10 ? 100000LL * n : 1000000LL * (n - 9));
            EXPECT_GE(phase[j].t - due, 0) << "repeat " << n << " of the change at " << c - t0;
            EXPECT_LT(phase[j].t - due, 6000) << "repeat " << n << " of the change at " << c - t0;
        }
        i = j;
    }
    EXPECT_EQ(changes, 3);   // the first key, the launch, the burnout
    EXPECT_NEAR(count[kAttitude], 1000, 1);
    EXPECT_NEAR(count[kNavState], 200, 1);
    EXPECT_NEAR(count[kGeometry], 10, 1);
    EXPECT_NEAR(count[kPhase], 10 + 10 * changes, 2);
    // Within the attitude's 10 ms, and the pass that finds it due (1 ms, up
    // to 0.5 ms late)
    EXPECT_LE(worst_delay_us, 10000.0 + 1500.0);
    EXPECT_GT(worst_delay_us, 0.0);
}

TEST(AidSchedule, EachKeepsToItsOwnPass)
{
    // Passes 1 ms apart and on time, one phase key: after the first two (the
    // key, then the attitude it held up), each frame goes on the pass it
    // falls due, at its offset from the attitude's: the state 5 ms, the
    // phase's repeats 3 ms (from the key's pass), the geometry 7 ms.
    AidSchedule s;
    std::vector<Sent> out;
    const int64_t t0 = 1000000;
    passes(s, t0, t0 + 3000000, out);
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
            EXPECT_EQ((out[i].t - t0) % 10000, offset[out[i].sub]) << i << ": sub-type " << int(out[i].sub);
        }
    }
    EXPECT_EQ(count[kAttitude], 300);
    EXPECT_EQ(count[kNavState], 60);
    EXPECT_EQ(count[kPhase], 1 + 10 + 1);   // the key, its ten repeats, the first a second later
    EXPECT_EQ(count[kGeometry], 3);
}

TEST(AidSchedule, AfterAStallEachGoesOnceInItsOrder)
{
    // A second on time, then no pass for 15 ms over the attitude's, the
    // phase's, the state's and the geometry's times (1.000, 1.003, 1.005 and
    // 1.007 s on): they go one a pass in the order of priority, and the
    // attitude, more than its period late, restarts from its pass: the next
    // 10 ms after it, not on the next pass.
    AidSchedule s;
    std::vector<Sent> out;
    const int64_t t0 = 1000000;
    passes(s, t0, t0 + 1000000, out);
    out.clear();
    passes(s, t0 + 1015000, t0 + 1050000, out);
    ASSERT_GE(out.size(), 4u);
    const uint8_t order[4] = {kAttitude, kNavState, kPhase, kGeometry};
    for (int i = 0; i < 4; i++)
    {
        EXPECT_EQ(out[i].sub, order[i]) << i;
        EXPECT_EQ(out[i].t - t0, 1015000 + 1000 * i) << i;
    }
    std::vector<int64_t> attitude;
    for (const Sent& f : out)
        if (f.sub == kAttitude) attitude.push_back(f.t - t0);
    EXPECT_EQ(attitude, (std::vector<int64_t>{1015000, 1025000, 1035000, 1045000}));
}

TEST(AidSchedule, ALongStallBringsNoBurst)
{
    // 200 ms with no pass: each sub-type due goes once when the passes come
    // back, then each keeps its rate from there.
    AidSchedule s;
    std::vector<Sent> out;
    passes(s, 1000000, 3000000, out);
    out.clear();
    passes(s, 3200000, 3300000, out);
    int count[5] = {};
    for (const Sent& f : out) count[f.sub]++;
    EXPECT_EQ(count[kAttitude], 10);   // 100 ms at 100 Hz, the first at once
    EXPECT_LE(count[kNavState], 3);
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

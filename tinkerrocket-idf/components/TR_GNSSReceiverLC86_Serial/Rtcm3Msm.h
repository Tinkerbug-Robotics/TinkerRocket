// Rtcm3Msm.h — RTCM 10403.3 frame check and MSM7 decoding (pure, host-compilable)
//
// The LC86G has no binary navigation protocol, but $PAIR432,1 makes it emit
// RTCM3 MSM7 raw measurements beside its NMEA: per satellite and signal, the
// pseudorange, the carrier phase range, the phase-range rate (its Doppler, in
// m/s), C/N0 to 1/16 dB-Hz and a lock-time counter. This unit turns one
// message body into those numbers; Lc86Parser pulls the frames out of the
// mixed byte stream and gathers a receiver epoch's messages. Like the parser
// it has no ESP-IDF dependency, so tests_cpp/test_lc86_parser.cpp pins it
// against frames the Beetle's own module sent.
//
// What the module puts in them, measured on the bench (2026-09-25 to 27,
// gnss/cocom-rig/sdr/results/README.md):
//   - one epoch a second whatever the fix rate (checked at 1, 5 and 10 Hz):
//     GPS, GLONASS, Galileo, BeiDou, then QZSS, which alone clears the
//     multiple-message bit. A constellation with nothing tracked still sends
//     its (empty) message;
//   - one signal per satellite: L1 C/A, GLONASS L1OF, Galileo E1, BeiDou B1I;
//   - the phase-range rate has the receiver's own clock drift already taken
//     out (0.0 +- 0.2 ppb on the rig); the pseudorange keeps the clock bias.
//
// Field references are the RTCM 10403.3 data fields (DFnnn).
#ifndef RTCM3MSM_H
#define RTCM3MSM_H

#include <cstdint>
#include <cstddef>

namespace rtcm3
{

static constexpr uint8_t  kPreamble      = 0xD3;
static constexpr size_t   kMaxPayload    = 1023;   // 10-bit length field
static constexpr size_t   kFrameOverhead = 6;      // preamble + length (3) + CRC (3)
static constexpr uint32_t kInvalidTow    = 0xFFFFFFFFu;

// CRC-24Q over `len` bytes (the frame from the preamble through the payload).
uint32_t crc24q(const uint8_t* data, size_t len);

// The 12-bit message number at the start of a payload; 0 if too short.
uint16_t messageType(const uint8_t* payload, size_t len);

// UBX gnssId of an MSM7 message type (the numbering GNSSSatBlock uses):
// 0 GPS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS. 0xFF for anything else.
uint8_t msm7GnssId(uint16_t msg_type);

// True for the broadcast-ephemeris messages ($PAIR436,1): 1019 GPS, 1020
// GLONASS, 1042 BeiDou, 1044 QZSS, 1046 Galileo I/NAV.
bool isEphemeris(uint16_t msg_type);

// DF407 lock-time indicator -> minimum lock time, ms (saturates at 67108864).
uint32_t lockTimeMs(uint16_t indicator);

// One satellite-signal of an MSM7 message. A field the receiver marked
// invalid reads NaN (the floats) or is flagged below; nothing is guessed.
struct MsmCell
{
    double   pseudorange_m;   // DF397 + DF398 + DF405; NaN when invalid
    double   phase_range_m;   // DF397 + DF398 + DF406; NaN when invalid
    float    rate_mps;        // DF399 + DF404, phase-range rate; NaN when invalid
    float    cn0_dbhz;        // DF408; 0 = not reported
    uint32_t lock_ms;         // DF407 as a minimum lock time
    uint8_t  gnss_id;         // UBX gnssId, as msm7GnssId()
    uint8_t  sv_id;           // DF394 slot: the PRN (GPS, Galileo, BeiDou),
                              // the slot (GLONASS), PRN - 192 (QZSS)
    uint8_t  sig_id;          // DF395 slot: 2 = L1 C/A, L1OF, E1 C, B1I
    uint8_t  ext_info;        // DF419: GLONASS frequency channel + 7 (15 = unknown)
    bool     half_cycle;      // DF420: half-cycle ambiguity unresolved
};

// What decodeMsm7() learned from the message header.
struct MsmHeader
{
    uint16_t msg_type;
    uint8_t  gnss_id;
    uint32_t epoch_field;     // the raw 30-bit epoch time (DF004/DF248/DF427/DF428,
                              // or DF416 + DF034 for GLONASS)
    uint32_t gps_tow_ms;      // that epoch as GPS time of week, ms; kInvalidTow for
                              // GLONASS, whose epoch is UTC(SU) and needs leap seconds
    bool     multiple;        // DF393: more MSM messages follow for this epoch
    uint8_t  n_sats;
    uint8_t  n_cells;
};

// Read just the header of an MSM7 payload (n_sats and n_cells are left 0):
// enough to tell which epoch a message belongs to before decoding it.
// Returns false if this is not an MSM7 or is too short for its header.
bool peekMsm7(const uint8_t* payload, size_t len, MsmHeader& hdr);

// Decode an MSM7 payload (the bytes between the length and the CRC). The
// message's cells are written to `cells` up to `cap`; any beyond are counted
// in *dropped. Returns false, writing nothing, if this is not an MSM7 or its
// length does not match its own masks.
bool decodeMsm7(const uint8_t* payload, size_t len, MsmHeader& hdr,
                MsmCell* cells, size_t cap, size_t* dropped);

// The most cells one receiver epoch keeps, across every constellation. The
// richest epoch in an hour of open sky on the Beetle (2026-09-25) had 29.
static constexpr size_t kMaxEpochCells = 48;

// All of one receiver epoch's MSM7 messages.
struct MsmEpoch
{
    uint32_t gps_tow_ms;      // from the first non-GLONASS message; kInvalidTow if none
    uint8_t  n_cells;
    uint8_t  n_msgs;          // MSM7 messages folded in
    uint8_t  gnss_mask;       // bit (1 << gnss_id) per constellation folded in
    uint8_t  dropped;         // cells past kMaxEpochCells (saturates at 255)
    bool     complete;        // closed by a message with DF393 = 0, not by the next epoch
    MsmCell  cells[kMaxEpochCells];
};

}  // namespace rtcm3

#endif

// Lc86Parser.h — Quectel LC86G NMEA/PQTM stream parser (pure, host-compilable)
//
// Bytes in, GNSSData out. This unit is deliberately free of every ESP-IDF /
// FreeRTOS dependency so the checksum rules, the $PQTMPVT field mapping
// (including the vel_u = -VelD sign the EKF's life depends on), the $PQTMEPE
// accuracy merge, and the framing robustness are all pinned by host tests
// (tests_cpp/test_lc86_parser.cpp). The ESP UART glue lives in
// TR_GNSSReceiverLC86_Serial.cpp — keep it there.
//
// Protocol: Quectel LC26G/LC76G/LC86G GNSS Protocol Specification V1.4.
// Framing:  $<Address>[,<Data>]*<Checksum><CR><LF>. Checksum is the 8-bit XOR
//           of every character between (not including) '$' and '*', commas
//           included (the doc's Ql_Check_XOR reference implementation).
#ifndef LC86PARSER_H
#define LC86PARSER_H

#include <cstdint>
#include <cstddef>

#include <RocketComputerTypes.h>

#include "Rtcm3Msm.h"

namespace lc86
{

// What (if anything) completed on this byte.
enum class Event : uint8_t
{
    NONE = 0,     // byte consumed; no complete sentence yet
    PVT,          // $PQTMPVT parsed — pvt() holds the new epoch
    EPE,          // $PQTMEPE parsed — accuracies cached for the next PVT
    GGA,          // valid GGA — ggaQuality()/ggaNumSats() refreshed
    GSV,          // valid GSV — one sentence of a satellites-in-view burst
                  // folded into the table; see takeSat()
    PAIR_ACK,     // $PAIR001 — ackCommandId()/ackResult()
    NAV_MODE,     // $PAIR081,<NavMode> — navMode()
    RTCM_MODE,    // $PAIR433,<Mode> — rtcmMode()
    RTCM_EPH_MODE, // $PAIR437,<Enable> — rtcmEphMode()
    QTM_OK,       // $PQTMCFGMSGRATE,OK (write accepted)
    QTM_ERROR,    // $PQTMCFGMSGRATE,ERROR,<code> — qtmErrorCode()
    MSM_EPOCH,    // an RTCM3 MSM7 epoch closed — see takeMsmEpoch()
    RTCM,         // any other RTCM3 frame whose CRC checks (MSM7 folded in, or
                  // a message type this parser does not decode)
                  // After either of these two, rtcmFrame() holds the frame.
    OTHER_VALID,  // any other sentence with a valid checksum (liveness proof)
    BAD,          // complete line or RTCM3 frame discarded (checksum/format/overflow)
};

// 8-bit XOR over `len` bytes (the span between '$' and '*').
uint8_t checksum(const char* data, size_t len);

// Frame `body` as "$<body>*HH\r\n" (checksum appended, NUL-terminated).
// Returns the frame length written (excluding the NUL), 0 if out_cap is
// too small.
size_t buildSentence(const char* body, char* out, size_t out_cap);

class Lc86Parser
{
public:
    // Feed one received byte; returns what (if anything) completed. Line
    // reassembly is internal, so bytes may arrive in arbitrary slices
    // (pollNewPVT drains bounded chunks — sentences routinely straddle calls).
    //
    // RTCM3 frames ($PAIR432) share the stream. NMEA is 7-bit ASCII, so a
    // 0xD3 can only be a frame's preamble: from there the frame's own length
    // says how many bytes are binary, and only a frame whose CRC-24Q checks
    // is decoded. A 0xD3 whose next bytes cannot be a header is dropped and
    // the byte that gave it away is read as NMEA again, so stray noise costs
    // at most the bytes before the check that caught it.
    Event feed(uint8_t byte);

    // Drop any partially-assembled line or RTCM3 frame. Call after a baud
    // change so cross-baud garbage can't glue onto the front of the next real
    // sentence. Cached EPE values, an open MSM7 epoch and counters survive.
    void resetLine();

    // Valid after Event::PVT. Every GNSSData field is filled except time_us
    // (left 0 — the owner stamps MCU micros at parse time, matching the
    // u-blox driver's semantics); h_acc_m/v_acc_m carry the most recent
    // $PQTMEPE values (255 = never seen / unknown).
    const GNSSData& pvt() const { return pvt_; }

    // GPS time of the last $PQTMPVT: its <TOW> (ms into the week), and the
    // GPS week, which the sentence never states. The week comes from its UTC
    // <Date>/<Time> plus <LeapS>, then the one whose time of week matches
    // <TOW> (so a UTC Saturday night that GPS has already turned into the
    // next week is named right). 0 = no time yet (fixless, no date).
    uint32_t pvtTowMs() const { return pvt_tow_ms_; }
    uint16_t pvtGpsWeek() const { return pvt_gps_week_; }

    // The last $PQTMPVT's <Sep>: the geoid's height above the WGS84
    // ellipsoid there, m. <Alt> (GNSSData alt_mm) is above the geoid (MSL);
    // alt + sep is the ellipsoidal height a raw-GNSS filter works in (#1600).
    // False until a fix states one.
    bool geoidSepM(float& sep_m) const { sep_m = geoid_sep_m_; return geoid_sep_valid_; }

    // Valid after Event::PAIR_ACK ($PAIR001,<CommandID>,<Result>).
    // Result: 0 ok, 1 being processed, 2 send failed, 3 unsupported,
    // 4 parameter error, 5 service busy (protocol spec §2.4.1).
    uint16_t ackCommandId() const { return ack_cmd_; }
    uint8_t  ackResult() const { return ack_result_; }

    // Valid after Event::QTM_ERROR.
    int qtmErrorCode() const { return qtm_err_; }

    // Valid after Event::NAV_MODE: 0 Normal, 1 Fitness, 3 Balloon,
    // 4 Stationary, 5 Drone, 7 Swimming (protocol spec §2.4.25).
    uint8_t navMode() const { return nav_mode_; }

    // Valid after Event::RTCM_MODE: -1 off, 0 MSM4, 1 MSM7 ($PAIR433, the
    // read-back of $PAIR432).
    int8_t rtcmMode() const { return rtcm_mode_; }

    // Valid after Event::RTCM_EPH_MODE: 0 off, 1 on ($PAIR437, the read-back
    // of $PAIR436: RTCM3 ephemeris messages 1019/1020/1042/1044/1046).
    int8_t rtcmEphMode() const { return rtcm_eph_mode_; }

    // Per-satellite report (#1032): moves a completed satellites-in-view
    // burst into `out` and clears the ready flag; false when none is waiting.
    //
    // GSV is the only C/N0 source this part has — it speaks no UBX, so there
    // is no NAV-SAT.  What that costs, stated so nobody reads more out of the
    // record than is in it:
    //   - cno_dbhz, elev_deg and azim_2deg are real, straight off the wire.
    //     An empty elevation/azimuth field (a satellite merely being searched
    //     for) lands as 0, not as a sentinel — GSV has none.
    //   - One signal per satellite. BeiDou entries are B1I; a B1C set is
    //     skipped and counted (gsvSkipped()), because nothing in the record
    //     could say which signal a C/N0 came from.
    //   - Entries are ordered highest elevation first, so when more satellites
    //     have signal than the record holds, the lowest ones are cut, from
    //     every constellation alike.
    //   - QZSS arrives on the GP talker and is recorded as gnssId 5, as a
    //     u-blox NAV-SAT would report it.
    //   - flags is ALWAYS 0.  GSV reports no used-in-solution bit, no quality
    //     indicator, no health and no ephemeris/almanac state; the used bit
    //     lives in GSA, which cannot be matched back to a GSV entry without
    //     the NMEA 4.10 <SystemID> field.  Zero here means "not reported",
    //     NOT "unused" or "unhealthy".
    //   - itow_ms is the TOW of the most recent $PQTMPVT, so the record pairs
    //     with that epoch's GNSSData exactly as the u-blox one does.  0
    //     before the first PVT.
    // time_us is left for the owner to stamp, matching pvt().
    bool takeSat(GNSSSatData& out);

    // Raw measurements (RTCM3 MSM7, $PAIR432,1): moves the latest closed
    // epoch into `out` and clears the ready flag; false when none is waiting.
    // An epoch closes on the message that clears DF393 (QZSS, on this
    // module) or, should that one be lost, when the next epoch's first
    // message arrives (complete = false then). msmEpoch() reads the same
    // epoch without taking it.
    bool takeMsmEpoch(rtcm3::MsmEpoch& out);
    const rtcm3::MsmEpoch& msmEpoch() const { return msm_ready_; }

    // The frame that produced the last Event::RTCM or Event::MSM_EPOCH,
    // verbatim from the 0xD3 through its CRC-24Q, for logging. Valid only
    // until the next feed(): the next frame reuses the buffer.
    const uint8_t* rtcmFrame() const { return rtcm_buf_; }
    size_t rtcmFrameLen() const { return rtcm_frame_len_; }

    // Valid after Event::GGA. Debug/liveness only — GGA is NEVER a fix
    // source here (no vertical velocity; see the PQTMPVT-is-load-bearing
    // rule in TR_GNSSReceiverLC86_Serial.cpp).
    uint8_t ggaQuality() const { return gga_quality_; }
    uint8_t ggaNumSats() const { return gga_num_sats_; }

    // Diagnostics (monotonic).
    uint32_t badLines() const { return bad_lines_; }            // checksum/format fails
    uint32_t overlongLines() const { return overlong_lines_; }  // dropped at kMaxLine
    uint32_t truncatedLines() const { return truncated_lines_; } // resynced on '$'
    uint32_t gsvSentences() const { return gsv_sentences_; }    // GSV lines folded in
    uint32_t gsvOverflows() const { return gsv_overflows_; }    // entries dropped at kMaxSatBuild
    uint32_t gsvSkipped() const { return gsv_skipped_; }        // BeiDou B1C lines left out
    uint32_t gsvBursts() const { return gsv_bursts_; }          // bursts closed, empty ones too
    uint32_t gsvBurstsWithSats() const { return gsv_bursts_with_sats_; } // ... that made a record
    uint32_t rtcmPreambles() const { return rtcm_preambles_; }  // 0xD3 bytes that opened a frame
    uint32_t rtcmFrames() const { return rtcm_frames_; }        // CRC-checked frames, any type
    uint32_t rtcmEphFrames() const { return rtcm_eph_frames_; } // ... that are ephemeris (1019/1020/1042/1044/1046)
    uint32_t rtcmCrcFails() const { return rtcm_crc_fails_; }   // full-length frames that failed
    uint32_t rtcmBadHeaders() const { return rtcm_bad_headers_; } // 0xD3 not followed by a header
    uint32_t msmMessages() const { return msm_messages_; }      // MSM7 messages decoded
    uint32_t msmMalformed() const { return msm_malformed_; }    // MSM7 type, length off its masks
    uint32_t msmEpochs() const { return msm_epochs_; }          // epochs closed
    uint32_t msmEpochsIncomplete() const { return msm_epochs_incomplete_; } // ... by the next one

private:
    // NMEA 0183 caps sentences at 82 chars but Quectel's $PQTMPVT runs ~137;
    // anything beyond this is garbage (or a baud mismatch) and is dropped.
    static constexpr size_t kMaxLine   = 200;
    static constexpr size_t kMaxFields = 24;

    Event parseLine();
    Event parsePqtmPvt(const char* const* f, size_t n);
    Event parsePqtmEpe(const char* const* f, size_t n);
    Event parsePairAck(const char* const* f, size_t n);
    Event parseNavMode(const char* const* f, size_t n);
    Event parseRtcmMode(const char* const* f, size_t n);
    Event parseRtcmEphMode(const char* const* f, size_t n);
    Event parseQtmCfgMsgRate(const char* const* f, size_t n);
    Event parseGga(const char* const* f, size_t n);
    Event parseGsv(const char* addr, const char* const* f, size_t n);

    // A GSV burst is one set of sentences per constellation, back to back.
    // It is closed by the NEXT non-GSV sentence rather than by counting sets:
    // nothing in the stream says which talker comes last, and the module
    // emits GGA and $PQTMPVT at the fix rate, so a burst is never the tail of
    // the stream in practice.
    void finalizeGsv();
    void dropTalker(uint16_t talker);

    // RTCM3 framing: feedRtcm() takes every byte from a 0xD3 to the end of
    // the frame its length gives; parseRtcm() checks and dispatches it.
    Event feedRtcm(uint8_t byte, bool& reread);
    Event parseRtcm();
    Event foldMsm7(const uint8_t* payload, size_t len);
    void  closeMsmEpoch(bool complete);

    char   line_[kMaxLine + 1];
    size_t line_len_ = 0;
    bool   in_line_  = false;

    uint8_t rtcm_buf_[rtcm3::kMaxPayload + rtcm3::kFrameOverhead];
    size_t  rtcm_len_    = 0;   // bytes held, preamble included
    size_t  rtcm_want_   = 0;   // whole frame length, once the header is in
    size_t  rtcm_frame_len_ = 0; // length of the last CRC-checked frame
    bool    in_rtcm_     = false;

    // The epoch being gathered and the last one closed (what takeMsmEpoch()
    // hands out). ~1.9 kB each; the parser lives inside the driver, which
    // the collector holds statically, so neither is ever on a task stack.
    rtcm3::MsmEpoch msm_build_ = {};
    rtcm3::MsmEpoch msm_ready_ = {};
    bool            msm_open_  = false;
    bool            msm_ready_flag_ = false;

    GNSSData pvt_ = {};
    uint8_t epe_h_acc_m_ = 255;  // $PQTMEPE EPE_2D, cached for the next PVT
    uint8_t epe_v_acc_m_ = 255;  // $PQTMEPE EPE_Down

    uint16_t ack_cmd_    = 0;
    uint8_t  ack_result_ = 0;
    int      qtm_err_    = 0;
    uint8_t  nav_mode_   = 0xFF;  // 0xFF until a $PAIR081 answer arrives
    int8_t   rtcm_mode_  = -2;    // -2 until a $PAIR433 answer arrives
    int8_t   rtcm_eph_mode_ = -2; // -2 until a $PAIR437 answer arrives
    uint8_t  gga_quality_  = 0;
    uint8_t  gga_num_sats_ = 0;

    // A four-constellation sky can show more satellites in view than the
    // record carries; collect generously and let gnssSatSelect() decide what
    // survives — it keeps every cno > 0 entry first, so truncation only ever
    // drops satellites with no signal.
    static constexpr uint8_t kMaxSatBuild = 40;

    GNSSSatBlock build_[kMaxSatBuild] = {};
    uint16_t    build_src_[kMaxSatBuild] = {};  // talker ID chars of each entry
    uint8_t     build_n_    = 0;
    bool        gsv_active_ = false;   // a burst is open, awaiting its closer
    GNSSSatData sat_        = {};
    bool        sat_ready_  = false;
    uint32_t    pvt_tow_ms_ = 0;       // $PQTMPVT <TOW>, ms into the GPS week
    uint16_t    pvt_gps_week_ = 0;     // from <Date>/<Time>/<LeapS>; 0 = unknown
    float       geoid_sep_m_ = 0.0f;   // $PQTMPVT <Sep>, m
    bool        geoid_sep_valid_ = false;

    uint32_t bad_lines_       = 0;
    uint32_t overlong_lines_  = 0;
    uint32_t truncated_lines_ = 0;
    uint32_t gsv_sentences_   = 0;
    uint32_t gsv_overflows_   = 0;
    uint32_t gsv_skipped_     = 0;
    uint32_t gsv_bursts_      = 0;
    uint32_t gsv_bursts_with_sats_ = 0;
    uint32_t rtcm_preambles_   = 0;
    uint32_t rtcm_frames_      = 0;
    uint32_t rtcm_eph_frames_  = 0;
    uint32_t rtcm_crc_fails_   = 0;
    uint32_t rtcm_bad_headers_ = 0;
    uint32_t msm_messages_     = 0;
    uint32_t msm_malformed_    = 0;
    uint32_t msm_epochs_       = 0;
    uint32_t msm_epochs_incomplete_ = 0;
};

}  // namespace lc86

#endif

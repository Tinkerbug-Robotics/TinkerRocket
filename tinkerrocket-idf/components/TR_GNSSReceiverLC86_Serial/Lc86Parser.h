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

namespace lc86
{

// What (if anything) completed on this byte.
enum class Event : uint8_t
{
    NONE = 0,     // byte consumed; no complete sentence yet
    PVT,          // $PQTMPVT parsed — pvt() holds the new epoch
    EPE,          // $PQTMEPE parsed — accuracies cached for the next PVT
    GGA,          // valid GGA — ggaQuality()/ggaNumSats() refreshed
    PAIR_ACK,     // $PAIR001 — ackCommandId()/ackResult()
    QTM_OK,       // $PQTMCFGMSGRATE,OK (write accepted)
    QTM_ERROR,    // $PQTMCFGMSGRATE,ERROR,<code> — qtmErrorCode()
    OTHER_VALID,  // any other sentence with a valid checksum (liveness proof)
    BAD,          // complete line discarded (checksum/format/overflow)
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
    Event feed(uint8_t byte);

    // Drop any partially-assembled line. Call after a baud change so
    // cross-baud garbage can't glue onto the front of the next real sentence.
    // Cached EPE values and counters survive.
    void resetLine();

    // Valid after Event::PVT. Every GNSSData field is filled except time_us
    // (left 0 — the owner stamps MCU micros at parse time, matching the
    // u-blox driver's semantics); h_acc_m/v_acc_m carry the most recent
    // $PQTMEPE values (255 = never seen / unknown).
    const GNSSData& pvt() const { return pvt_; }

    // Valid after Event::PAIR_ACK ($PAIR001,<CommandID>,<Result>).
    // Result: 0 ok, 1 being processed, 2 send failed, 3 unsupported,
    // 4 parameter error, 5 service busy (protocol spec §2.4.1).
    uint16_t ackCommandId() const { return ack_cmd_; }
    uint8_t  ackResult() const { return ack_result_; }

    // Valid after Event::QTM_ERROR.
    int qtmErrorCode() const { return qtm_err_; }

    // Valid after Event::GGA. Debug/liveness only — GGA is NEVER a fix
    // source here (no vertical velocity; see the PQTMPVT-is-load-bearing
    // rule in TR_GNSSReceiverLC86_Serial.cpp).
    uint8_t ggaQuality() const { return gga_quality_; }
    uint8_t ggaNumSats() const { return gga_num_sats_; }

    // Diagnostics (monotonic).
    uint32_t badLines() const { return bad_lines_; }            // checksum/format fails
    uint32_t overlongLines() const { return overlong_lines_; }  // dropped at kMaxLine
    uint32_t truncatedLines() const { return truncated_lines_; } // resynced on '$'

private:
    // NMEA 0183 caps sentences at 82 chars but Quectel's $PQTMPVT runs ~137;
    // anything beyond this is garbage (or a baud mismatch) and is dropped.
    static constexpr size_t kMaxLine   = 200;
    static constexpr size_t kMaxFields = 24;

    Event parseLine();
    Event parsePqtmPvt(const char* const* f, size_t n);
    Event parsePqtmEpe(const char* const* f, size_t n);
    Event parsePairAck(const char* const* f, size_t n);
    Event parseQtmCfgMsgRate(const char* const* f, size_t n);
    Event parseGga(const char* const* f, size_t n);

    char   line_[kMaxLine + 1];
    size_t line_len_ = 0;
    bool   in_line_  = false;

    GNSSData pvt_ = {};
    uint8_t epe_h_acc_m_ = 255;  // $PQTMEPE EPE_2D, cached for the next PVT
    uint8_t epe_v_acc_m_ = 255;  // $PQTMEPE EPE_Down

    uint16_t ack_cmd_    = 0;
    uint8_t  ack_result_ = 0;
    int      qtm_err_    = 0;
    uint8_t  gga_quality_  = 0;
    uint8_t  gga_num_sats_ = 0;

    uint32_t bad_lines_       = 0;
    uint32_t overlong_lines_  = 0;
    uint32_t truncated_lines_ = 0;
};

}  // namespace lc86

#endif

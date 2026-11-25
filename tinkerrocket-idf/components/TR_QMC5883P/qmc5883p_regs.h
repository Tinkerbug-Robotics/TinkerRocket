#ifndef QMC5883P_REGS_H
#define QMC5883P_REGS_H

#include <stdint.h>

// QST QMC5883P register map and bit-field packing (#1312).
//
// Header-only and IDF-free so the packing can be pinned on the host against
// the datasheet's own worked examples (tests_cpp/test_qmc5883p_regs.cpp).
// Source: QMC5883P Datasheet Rev. C, QST document 13-52-19 — sections 7
// (application examples) and 9 (registers).  TR_QMC5883P.h wraps this in
// the I2C driver proper.
namespace qmc5883p {

// --- I2C ---
// Fixed 7-bit address (datasheet 5.4: "The default I2C address for QMC5883P
// is 2CH.  If more I2C address options are required, please contact
// factory").  The LGA-16 has no address-select pin.  Standard and fast mode
// (100 / 400 kHz), Table 8.
static constexpr uint8_t I2C_ADDR = 0x2C;

// --- Register addresses (Table 14) ---
static constexpr uint8_t REG_CHIP_ID = 0x00;   // reads CHIP_ID
static constexpr uint8_t REG_XOUT_L  = 0x01;   // X LSB — the 6-byte burst starts here
static constexpr uint8_t REG_XOUT_H  = 0x02;
static constexpr uint8_t REG_YOUT_L  = 0x03;
static constexpr uint8_t REG_YOUT_H  = 0x04;
static constexpr uint8_t REG_ZOUT_L  = 0x05;
static constexpr uint8_t REG_ZOUT_H  = 0x06;
static constexpr uint8_t REG_STATUS  = 0x09;
static constexpr uint8_t REG_CTRL1   = 0x0A;
static constexpr uint8_t REG_CTRL2   = 0x0B;
// Absent from the register map table, yet the first write of every setup
// example in section 7: "Write Register 29H by 0x06 (Define the sign for X
// Y and Z axis)".
//
// The driver writes 0x00, the value the register reads after a reset, and
// NOT the datasheet's 0x06.  Until the soft-reset fix in TR_QMC5883P.cpp,
// the 0x06 write never landed: it went in while SOFT_RST still held the
// part, and the boot readback read 0x29 = 0x00.  Every QMC5883P log so far
// was taken at 0x00, and so were the chip signs #1590 measured on the Beetle
// (+1, -1, -1 against Figure 2, magTypeChipSign in RocketComputerTypes.h).
// Those signs are the 0x00 frame.  0x06 would most likely flip Y and Z into
// Figure 2's frame: bits 1 and 2 are set, and Y and Z are exactly the axes
// #1590 found reversed.  That is inferred, not bench-tested.  Writing 0x06
// would reverse the Beetle's pad heading again.  Keep 0x00 unless the signs
// change with it.
static constexpr uint8_t REG_AXIS_SIGN   = 0x29;
static constexpr uint8_t AXIS_SIGN_VALUE = 0x00;
static constexpr uint8_t AXIS_SIGN_DATASHEET_EXAMPLE = 0x06;   // never written

static constexpr uint8_t CHIP_ID = 0x80;       // 9.2.1: "The default value is 80H"

// --- STATUS (0x09), 9.2.2 ---
// DRDY: all three axes have been loaded into the output registers.  "It is
// reset to 0 by reading the status register" — NOT by reading the data — so
// a status read is a deliberate act in the driver, never a free peek.
// OVFL: an axis went past ±30000 LSB (±8 G is ±30000 counts at 3750 LSB/G);
// reset to 0 after the bit is read.
static constexpr uint8_t STATUS_DRDY = 1u << 0;
static constexpr uint8_t STATUS_OVFL = 1u << 1;

// One burst from XOUT_L through STATUS: the six axis bytes, the two unmapped
// bytes at 07H-08H (they read 0x00), then STATUS at index 8. The register
// pointer runs on past 06H into STATUS, and a STATUS byte read this way clears
// DRDY exactly as the single-register read does. Not stated by the datasheet,
// which only shows 01H-06H bursts; bench-verified on the mini (#1485, 100
// trials): the ninth byte equals a plain STATUS read, and a STATUS read right
// after the burst finds DRDY clear. STATUS bits 3 and 4 are undocumented and
// read 1 on that part (0x19 with DRDY, 0x18 without) — mask, never compare.
static constexpr uint8_t BURST_WITH_STATUS_LEN = REG_STATUS - REG_XOUT_L + 1;   // 9
static constexpr uint8_t BURST_STATUS_INDEX    = REG_STATUS - REG_XOUT_L;       // 8
static_assert(BURST_WITH_STATUS_LEN == 9 && BURST_STATUS_INDEX == 8,
              "the status byte is the ninth of a burst from XOUT_L");

// --- CTRL1 (0x0A), Table 17 ---
//   [7:6] OSR2  down-sampling depth   00=1  01=2  10=4  11=8
//   [5:4] OSR1  over-sample ratio     00=8  01=4  10=2  11=1   (00 is the MOST filtering)
//   [3:2] ODR   output data rate      00=10 Hz  01=50 Hz  10=100 Hz  11=200 Hz
//   [1:0] MODE  00=suspend  01=normal  10=single  11=continuous
// Normal mode measures continuously AT the ODR (6.2.1).  Continuous mode
// "runs all the time without sleep time" — the ODR bits stop mattering, the
// part free-runs at up to 1.5 kHz and draws 2.2 mA (Table 2, 6.2.3).  Single
// mode measures once and drops back to suspend.
enum class Mode : uint8_t { SUSPEND = 0, NORMAL = 1, SINGLE = 2, CONTINUOUS = 3 };
enum class Odr  : uint8_t { HZ_10 = 0, HZ_50 = 1, HZ_100 = 2, HZ_200 = 3 };
enum class Osr1 : uint8_t { X8 = 0, X4 = 1, X2 = 2, X1 = 3 };
enum class Osr2 : uint8_t { X1 = 0, X2 = 1, X4 = 2, X8 = 3 };

// --- CTRL2 (0x0B), Table 18 ---
//   [7]   SOFT_RST   1 = "restore default value of all registers" (then suspend)
//   [6]   SELF_TEST  1 = inject the self-test field (continuous mode only)
//   [3:2] RNG        00=±30 G  01=±12 G  10=±8 G  11=±2 G
//   [1:0] SET/RESET  00=set and reset on  01=set only on  1x=set and reset off
// Set/reset is the part's periodic degauss — "in SET ONLY ON or SET AND
// RESET OFF mode, the offset is not renewed during measuring" — i.e. the
// counterpart of the IIS2MDC's OFF_CANC with periodic Set pulses.
//
// SOFT_RST does not clear itself.  Table 18 says SELF_TEST is "auto clear
// after the data is updated" and says nothing like that for SOFT_RST.  The
// Linux QMC6308 driver (same family, same CTRL2 layout) puts it plainly:
// "The SOFT_RST bit is not auto-cleared and must be written back 0".  While
// the bit is set, writes do not stick.  The Beetle's boot readback until
// this fix was CTRL1=0xC9 CTRL2=0x00 AXIS_SIGN=0x00.  configure() had
// written 0x29=0x06, CTRL1=0xC8, CTRL2=0x08 and CTRL1=0xC9 after a 0x80
// reset, and only the final CTRL1 write stuck.  The CTRL2 write cleared the
// bit, but its own RNG did not stick.  So the part ran at ±30 G (1000 LSB/G)
// under the ±8 G conversion (3750 LSB/G), and |B| read 3.75x low: #1590's
// open scale question.  CTRL2 is Read/Write (Table 14), so that readback is
// the register.
enum class Range    : uint8_t { G30 = 0, G12 = 1, G8 = 2, G2 = 3 };
enum class SetReset : uint8_t { SET_AND_RESET_ON = 0, SET_ONLY_ON = 1, OFF = 2 };
static constexpr uint8_t CTRL2_SOFT_RST  = 1u << 7;
static constexpr uint8_t CTRL2_SELF_TEST = 1u << 6;
// The CTRL2 bits a readback compares: SOFT_RST and SELF_TEST must read 0,
// RNG and SET/RESET as written.  Bits 5:4 are RFU and masked.
static constexpr uint8_t CTRL2_READBACK_MASK = 0xCF;

constexpr uint8_t ctrl1(Mode mode, Odr odr, Osr1 osr1, Osr2 osr2)
{
    return (uint8_t)(((uint8_t)osr2 << 6) | ((uint8_t)osr1 << 4) |
                     ((uint8_t)odr << 2) | (uint8_t)mode);
}

constexpr uint8_t ctrl2(Range range, SetReset set_reset)
{
    return (uint8_t)(((uint8_t)range << 2) | (uint8_t)set_reset);
}

// The range a CTRL2 value selects, i.e. the range the part converts at.
constexpr Range rangeOfCtrl2(uint8_t ctrl2_value)
{
    return (Range)((ctrl2_value >> 2) & 0x3u);
}

// What configure() reads back after its writes: the three registers it sets.
struct ConfigReadback {
    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t axis_sign;
};

// True iff the part holds what configure() wrote.  CTRL1 is compared whole
// (every bit is defined), CTRL2 under CTRL2_READBACK_MASK.
constexpr bool configTook(const ConfigReadback& rb, uint8_t want_ctrl1,
                          uint8_t want_ctrl2, uint8_t want_axis_sign)
{
    return rb.ctrl1 == want_ctrl1 &&
           (rb.ctrl2 & CTRL2_READBACK_MASK) == (want_ctrl2 & CTRL2_READBACK_MASK) &&
           rb.axis_sign == want_axis_sign;
}

// --- Scale (Table 2, "Sensitivity") ---
constexpr uint8_t fullScaleGauss(Range r)
{
    return r == Range::G30 ? 30 : r == Range::G12 ? 12 : r == Range::G8 ? 8 : 2;
}

constexpr uint16_t lsbPerGauss(Range r)
{
    return r == Range::G30 ? 1000
         : r == Range::G12 ? 2500
         : r == Range::G8  ? 3750
         :                   15000;
}

// 1 G = 100 µT.  At ±8 G this is the 100/3750 that MAG_TYPE_QMC5883P
// promises every log reader (RocketComputerTypes.h).
constexpr float uTPerLsb(Range r) { return 100.0f / (float)lsbPerGauss(r); }

// The range MAG_TYPE_QMC5883P's count scale is defined at.
static constexpr Range CONTRACT_RANGE = Range::G8;

// A count taken at `actual` converted to the CONTRACT_RANGE scale: rounded to
// the nearest count and saturated to int16, as the part saturates.  The
// identity at ±8 G.  The driver uses it only if CTRL2 will not take ±8 G, so
// the counts it hands out keep the scale every log reader assumes.  At ±30 G
// that is x3.75; a ±30 G count is 0.1 µT, so nothing a heading needs is lost.
inline int16_t toContractCounts(int16_t count, Range actual)
{
    const int32_t num = (int32_t)lsbPerGauss(CONTRACT_RANGE);
    const int32_t den = (int32_t)lsbPerGauss(actual);
    if (num == den) return count;
    const int32_t scaled = (int32_t)count * num;
    const int32_t q = (scaled >= 0) ? (scaled + den / 2) / den
                                    : -((-scaled + den / 2) / den);
    return q > 32767 ? (int16_t)32767 : q < -32768 ? (int16_t)-32768 : (int16_t)q;
}

constexpr uint32_t odrPeriodUs(Odr odr)
{
    return odr == Odr::HZ_10  ? 100000u
         : odr == Odr::HZ_50  ?  20000u
         : odr == Odr::HZ_100 ?  10000u
         :                        5000u;
}

// --- Data (9.2.1) ---
// "Each axis has 16-bit data width in 2's complement", LSB register first,
// saturating at -32768 / 32767.
inline int16_t decodeAxis(const uint8_t* le2)
{
    return (int16_t)(((uint16_t)le2[1] << 8) | le2[0]);
}

// The QMC5883P has no hard-iron OFFSET registers.  The driver subtracts the
// offset in software with the saturation the ST part's silicon has, so a
// consumer of the count stream cannot tell which chip corrected it.
inline int16_t subtractSaturating(int16_t raw, int16_t offset)
{
    const int32_t d = (int32_t)raw - (int32_t)offset;
    return d > 32767 ? (int16_t)32767 : d < -32768 ? (int16_t)-32768 : (int16_t)d;
}

}  // namespace qmc5883p

#endif  // QMC5883P_REGS_H

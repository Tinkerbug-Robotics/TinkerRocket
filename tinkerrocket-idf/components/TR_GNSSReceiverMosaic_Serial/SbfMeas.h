// SbfMeas.h — the mosaic-G5's raw measurements, MeasEpoch (4027) and MeasExtra (4000), read signal by signal (pure, host-compilable)
//
// MeasEpoch holds every tracked signal of one epoch in two levels: a Type1
// sub-block per satellite with its master signal in full, and N2 Type2
// sub-blocks after it with the satellite's other signals as offsets from the
// master. The signal a filter wants (GPS L1 C/A, Galileo E1, BeiDou B1I) sits
// at either level: on the 10/16 V9 captures GPS L1 C/A was a Type2 (under L5)
// in 46 % of the cone-off run's satellite-epochs and BeiDou B1I one in 14 % of
// the nose-cone run's, so a reader has to fold the Type2s back to whole
// signals. SbfMeasReader does that as an iterator, one signal a
// call, so nothing the size of a whole epoch lands on the GNSS poll task's
// stack, and it checks every sub-block against the block's Length before
// handing out the first.
//
// Reference: mosaic-G5 Reference Guide, firmware 1.0.1, 4.1.9 (SVIDs), 4.1.10
// (signal numbers), 4.2.1 (MeasEpoch, MeasExtra). Every field is read at the
// guide's offsets, sub-block sizes come from SB1Length/SB2Length/SBLength
// (never hard-coded), undefined SVIDs and signal numbers are skipped (4.1.12),
// and each do-not-use value is the guide's: a code or carrier is invalid only
// when BOTH its MSB and LSB parts hold theirs. Pinned by
// tests_cpp/test_sbf_meas.cpp; checked on the 10/16 V9 spin-rig captures
// behind TR_SBF_PRIVATE_CAPTURE.
#ifndef SBFMEAS_H
#define SBFMEAS_H

#include <cstddef>
#include <cstdint>

#include "SbfParser.h"

namespace mosaic
{

// The signals the raw-GNSS filter reads (4.1.10).
enum SbfSig : uint8_t
{
    SBF_SIG_GPS_L1CA = 0,
    SBF_SIG_GAL_E1   = 17,
    SBF_SIG_BDS_B1I  = 28,
};

// The constellation an SVID names (4.1.9).
enum class SbfSys : uint8_t { GPS, GLO, GAL, SBAS, BDS, QZS, NAVIC, LBAND, NONE };

// The system of an SVID and its number within it: the PRN (GPS, Galileo,
// BeiDou 141-180 -> C01-C40 and 223-245 -> C41-C63, QZSS, NavIC, SBAS), or
// the GLONASS slot (0 when the receiver does not know it, SVID 62). NONE for
// an SVID the guide leaves undefined.
SbfSys sbfSvid(uint8_t svid, uint8_t& prn);

// A GLONASS FDMA frequency number the receiver has not given.
static constexpr int8_t kGloFnUnknown = -128;

// Whether the guide defines signal number `sig` (0-39 less the reserved ones).
bool sbfSignalDefined(uint8_t sig);

// A signal's carrier frequency, Hz; 0 when the guide gives none (an undefined
// signal, the L-band beams, GLONASS L1/L2 without its frequency number).
// glo_fn is the GLONASS frequency number itself (-7..+6).
double sbfCarrierHz(uint8_t sig, int8_t glo_fn = kGloFnUnknown);

enum class SbfStatus : uint8_t
{
    OK,
    WRONG_BLOCK,   // not "$@" or not the block the reader reads
    SHORT,         // fewer bytes than the block's Length, or a Length too small for its header
    BAD_LAYOUT,    // sub-block lengths that cannot be, or sub-blocks running past Length
    SCRAMBLED,     // MeasEpoch CommonFlags bit 7: the receiver lacks the measurement permission
    NO_TIME,       // TOW or WNc do-not-use: the receiver has not set its time yet
};

// ── MeasEpoch (4027) ─────────────────────────────────────────────────────

struct SbfMeasHeader
{
    uint32_t tow_ms = kTowDnu;   // receiver time (GPS time plus the receiver's clock bias)
    uint16_t wnc = kWncDnu;
    uint8_t  revision = 0;
    uint16_t length = 0;
    uint8_t  n1 = 0;             // Type1 sub-blocks (satellites)
    uint8_t  sb1_len = 0;
    uint8_t  sb2_len = 0;
    uint8_t  common_flags = 0;   // b0 multipath mitigation, b1 smoothing, b3 clock steering, b5 high dynamics, b7 scrambled
    uint8_t  cum_clk_jumps = 0;  // ms, mod 256 (revision 1 on)
    bool     has_cum_clk_jumps = false;
};

// One signal of one satellite, whole: a Type1's master, or a Type2 folded
// onto its master. The code and carrier are kept as the receiver's integers,
// so nothing is lost before a caller picks its units.
struct SbfMeasSignal
{
    uint8_t  rx_channel = 0;
    uint8_t  svid = 0;
    uint8_t  sig = 0;             // signal number, with the SigIdxLo 31 extension applied
    uint8_t  antenna = 0;         // 0 main, 1 Aux1, 2 Aux2
    int8_t   glo_fn = kGloFnUnknown;   // GLONASS L1/L2 frequency number; a Type2 inherits its Type1's
    bool     master = false;      // from a Type1 sub-block
    bool     pr_valid = false;
    bool     dop_valid = false;
    bool     carrier_valid = false;
    bool     half_cycle = false;  // ObsInfo bit 2: the carrier has a half-cycle ambiguity
    bool     smoothed = false;    // ObsInfo bit 0: the code is smoothed
    uint8_t  cn0_raw = 255;       // 0.25 dB-Hz; 255 do-not-use
    uint16_t lock_s = 65535;      // continuous carrier, s; 65535 when there is no phase (a Type2's 255 too)
    int64_t  pr_mm = 0;           // pseudorange, mm
    int32_t  carrier_mcyc = 0;    // carrier phase less pr/lambda, millicycles
    double   doppler_hz = 0.0;    // positive for an approaching satellite; a Type2's is D1 * f2/f1 + its offset

    bool   cn0Valid() const { return cn0_raw != 255; }
    bool   lockValid() const { return lock_s != 65535; }
    // C/N0, dB-Hz: CN0 * 0.25, plus 10 for every signal but GPS L1P and L2P (1, 2).
    double cn0Dbhz() const { return cn0_raw * 0.25 + ((sig == 1 || sig == 2) ? 0.0 : 10.0); }
    double prM() const { return static_cast<double>(pr_mm) * 1.0e-3; }
    // The carrier phase as a range, m: L * lambda = PR + carrier_mcyc * 1e-3 * lambda.
    double phaseRangeM(double lambda_m) const { return prM() + carrier_mcyc * 1.0e-3 * lambda_m; }
};

class SbfMeasReader
{
public:
    // Check a whole MeasEpoch block (as SbfParser delivers it: CRC already
    // checked) and get ready to hand out its signals. Every sub-block is
    // checked against Length here, so next() never reads past it. header()
    // is filled whenever the header itself could be read.
    SbfStatus begin(const uint8_t* b, size_t len);

    const SbfMeasHeader& header() const { return hdr_; }

    // The next signal, in block order (each Type1, then its Type2s). False at
    // the end, or when begin() did not return OK.
    bool next(SbfMeasSignal& out);

    // Sub-blocks passed over so far: an undefined SVID (with its Type2s) or
    // signal number.
    uint16_t skipped() const { return skipped_; }

private:
    const uint8_t* b_ = nullptr;
    SbfMeasHeader  hdr_;
    bool           ok_ = false;
    size_t         t1_at_ = 0;      // the next Type1 sub-block
    uint8_t        t1_left_ = 0;
    size_t         t2_at_ = 0;      // the next Type2 of the current Type1
    uint8_t        t2_left_ = 0;
    uint16_t       skipped_ = 0;

    // The current Type1's master signal, which its Type2s are offsets from.
    struct Master
    {
        bool     defined = false;   // a defined SVID (its Type2s can be read)
        uint8_t  rx_channel = 0;
        uint8_t  svid = 0;
        int8_t   glo_fn = kGloFnUnknown;
        bool     pr_valid = false;
        int64_t  pr_mm = 0;
        bool     dop_valid = false;
        double   doppler_hz = 0.0;
        double   carrier_hz = 0.0;  // 0: unknown, so no Type2 Doppler can be scaled from it
    } m_;
};

// ── MeasExtra (4000) ─────────────────────────────────────────────────────

struct SbfExtraHeader
{
    uint32_t tow_ms = kTowDnu;   // the MeasEpoch it goes with has the same
    uint16_t wnc = kWncDnu;
    uint8_t  revision = 0;
    uint16_t length = 0;
    uint16_t n_sb = 0;           // sub-blocks: NrSB from N (mod 256), Length and SBLength
    uint8_t  sb_len = 0;
    float    doppler_var_factor = 0.0f;   // Hz^2 / cycle^2: Doppler variance [mHz^2] = CarrierVar * this
};

// One sub-block. MeasExtra's order is not MeasEpoch's (on the 10/16 captures
// it matched in 437 of 34,894 epochs): pair them by (rx_channel, sig,
// antenna), which is unique within a block. Fields the block's revision or
// SBLength does not carry have their has_ flag clear.
struct SbfExtraSignal
{
    uint8_t  rx_channel = 0;
    uint8_t  sig = 0;
    uint8_t  antenna = 0;
    int16_t  mp_correction_mm = 0;     // add to the pseudorange for the code without multipath mitigation
    int16_t  smoothing_corr_mm = 0;    // add to the pseudorange for the code without smoothing
    uint16_t code_var = 65535;         // 1e-4 m^2; 65535 do-not-use
    uint16_t carrier_var = 65535;      // mcycle^2; 65535 do-not-use
    uint16_t lock_s = 65535;           // s, not clipped at 254 as a Type2's is; 65535 do-not-use
    bool     has_cum_loss_cont = false;   // revision 1
    uint8_t  cum_loss_cont = 0;        // mod 256: up one at each relock or detected slip
    bool     has_car_mp_corr = false;  // revision 1
    int8_t   car_mp_corr = 0;          // 1/512 cycle
    bool     has_cn0_hires = false;    // revision 3
    uint8_t  cn0_hires = 0;            // C/N0 += this * 0.03125 dB-Hz
};

class SbfExtraReader
{
public:
    SbfStatus begin(const uint8_t* b, size_t len);
    const SbfExtraHeader& header() const { return hdr_; }
    bool next(SbfExtraSignal& out);
    uint16_t skipped() const { return skipped_; }   // sub-blocks with an undefined signal number

private:
    const uint8_t* b_ = nullptr;
    SbfExtraHeader hdr_;
    bool           ok_ = false;
    uint16_t       next_ = 0;
    uint16_t       skipped_ = 0;
};

// A Doppler's standard deviation as a range rate, m/s, from MeasExtra's
// carrier variance (mcycle^2) and its DopplerVarFactor; negative when the
// variance is do-not-use or the factor is not positive.
double sbfDopplerSigmaMps(uint16_t carrier_var, float doppler_var_factor, double lambda_m);

}  // namespace mosaic

#endif  // SBFMEAS_H

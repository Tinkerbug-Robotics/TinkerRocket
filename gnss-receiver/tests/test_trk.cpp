#include "sig_gen.h"

extern "C" {
#include "corr_float.h"
#include "gnss/trk.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

// Closed loop: the float correlator and one tracking channel on a synthetic signal.
TEST(Trk, PullsInLocksAndReadsTheBits)
{
    const double fs = 6.75e6, if_hz = 1.2e6, dop = 1500.0;
    const size_t spms = 6750, n = spms * 4000;  // 4 s
    siggen::Sat s;
    s.prn = 9;
    s.dop = dop;
    s.code_phase = 300.0;
    s.phase = 2.0;
    s.data_ms = 20;  // alternating bits: a transition at every bit edge
    auto x = siggen::make({s}, fs, if_hz, n, siggen::sigma_for(45.0, fs));

    auto cf = std::make_unique<corr_float_t>();
    corr_float_init(cf.get(), fs);
    const double kc = 4294967296.0 / fs, kk = double(uint64_t(1) << CORR_CODE_FRAC_BITS) / fs;
    int32_t if_word = int32_t(std::llround(if_hz * kc));
    uint64_t code0 = uint64_t(std::llround(1.023e6 * kk));
    trk_ch_t c;
    trk_start(&c, 9, float(dop + 60.0), 0.25f, if_word, code0, float(kc), float(kk));  // 60 Hz off
    corr_cmd_t st{};
    st.type = CORR_CMD_START;
    st.ch = 0;
    st.sig = GNSS_SIG_GPS_L1CA;
    st.prn = 9;
    st.t_start = 0;
    st.code_phase = uint64_t(std::llround(300.0 * double(uint64_t(1) << CORR_CODE_FRAC_BITS)));
    st.tap_offset = uint64_t(0.25 * double(uint64_t(1) << CORR_CODE_FRAC_BITS));
    trk_words(&c, &st.carr_word, &st.code_word);
    corr_float_command(cf.get(), &st);

    uint64_t last_t = 0;
    int nbits = 0, alternations = 0, prev_bit = 0;
    float locked_at = -1.0f;
    for (uint64_t t0 = 0; t0 < n; t0 += spms) {
        corr_dump_t d[4];
        int nd = corr_float_process(cf.get(), t0, x.data() + 2 * t0, spms, d, 4);
        for (int k = 0; k < nd; k++) {
            float T = float(double(d[k].t_samp - last_t) / fs);
            last_t = d[k].t_samp;
            if (d[k].seq == 0) {
                continue;
            }
            int bit;
            uint32_t bp;
            if (trk_update(&c, &d[k], T, &bit, &bp)) {
                if (nbits > 0 && bit != prev_bit) {
                    alternations++;
                }
                prev_bit = bit;
                nbits++;
            }
            if (locked_at < 0.0f && c.state == TRK_LOCKED) {
                locked_at = float(double(t0) / fs);
            }
            corr_cmd_t nc{};
            nc.type = CORR_CMD_NCO;
            nc.ch = 0;
            trk_words(&c, &nc.carr_word, &nc.code_word);
            corr_float_command(cf.get(), &nc);
        }
    }
    EXPECT_EQ(c.state, TRK_LOCKED);
    EXPECT_GT(locked_at, 0.0f);
    EXPECT_LT(locked_at, 1.5f);
    EXPECT_NEAR(c.dop_hz, dop, 3.0);
    EXPECT_NEAR(c.cn0, 45.0, 1.5);
    EXPECT_EQ(c.bit_sync, 1);
    EXPECT_GT(nbits, 50);
    EXPECT_EQ(alternations, nbits - 1);  // every decided bit flips: no bit errors at 45 dB-Hz
}

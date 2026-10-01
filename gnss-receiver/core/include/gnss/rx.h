/*
 * The receiver: channels, acquisition hand-over, the 1 ms tick, navigation
 * data, observables and PVT. This is the code the P4 will run; the host runner
 * (host/gnssrx.c) drives it exactly as the FPGA's interrupt will.
 *
 *   rx_tick()      every 1 ms tick, with the dumps since the last one; returns
 *                  correlator commands.
 *   rx_wants_snapshot() / rx_acquire()   acquisition on a raw-sample snapshot.
 *   rx_measure()   observables and PVT at a chosen sample.
 *
 * Time on the correlator side is the sample count (fs); receiver time maps it
 * to GPS time once the first fix has set the clock.
 */
#ifndef GNSS_RX_H
#define GNSS_RX_H

#include <stddef.h>
#include <stdint.h>

#include "gnss/acq.h"
#include "gnss/corr_if.h"
#include "gnss/eph.h"
#include "gnss/lnav.h"
#include "gnss/pvt.h"
#include "gnss/sig.h"
#include "gnss/trk.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double fs;                  /* correlator sample rate */
    double if_hz;               /* where L1 sits in the stream */
    float acq_threshold;        /* acquisition metric (peak / grid mean) for a detection */
    int acq_ms;                 /* snapshot length */
    float acq_interval_s;       /* how often to search for satellites not in a channel */
    float tap_chips;            /* early/late offset */
    int max_ch;                 /* channels to use, <= CORR_MAX_CH */
    uint32_t cmd_lead;          /* NCO commands computed from dump s are tagged s + cmd_lead (corr_if.h) */
    float hatch_s;              /* carrier smoothing of pseudoranges: time constant, s (0 = off) */
    int pvt_weights;            /* 1: weigh the fix by each measurement's sigma; 0: equal weights */
    float adapt_tau_s;          /* each satellite's own residual spread, learnt over this long, inflates
                                   its sigma (up to 5x) where the model is too kind to it (0 = off) */
    trk_profile_t quiet;        /* tracking loops at rest */
    trk_profile_t boost;        /* and under the boost's dynamics (rx_set_boost) */
} rx_cfg_t;

/* Per-satellite record of an epoch (what goes to RINEX and the logs). */
typedef struct {
    int sys;                    /* gnss_sys_t */
    int sig;                    /* gnss_sig_t tracked */
    int prn;
    int ch;
    double pr;                  /* m, carrier-smoothed while the PLL holds (rx_cfg_t.hatch_s) */
    double pr_raw;              /* m, as measured */
    double adr;                 /* carrier phase, cycles, RINEX sign (grows with range) */
    double dop;                 /* Hz */
    double t_sv;                /* transmit time, s of week (satellite clock, in its system's time) */
    float cn0;                  /* dB-Hz */
    float lock_s;               /* time since the PLL locked; 0 when it is not (the carrier phase is void) */
    int half_cycle;             /* the Costas half-cycle ambiguity is resolved */
} rx_obs_t;

/*
 * The Doppler observable is the NCO's mean frequency over the last RX_DOP_DUMPS code periods,
 * from the exact phase, moved forward by the loop's own rate for the half window it lags. A
 * wide boost loop jitters its NCO word by tens of hertz; the phase it holds is far steadier.
 */
#define RX_DOP_DUMPS 20

typedef struct {
    uint64_t t_start;           /* sample the channel started at */
    uint64_t last_t;            /* t_samp of the last dump, extended to 64 bits */
    uint32_t period;            /* seq of the last dump, extended to 32 bits */
    uint64_t last_code_phase;
    uint32_t last_carr_phase;
    uint32_t last_carr_cycles;
    int64_t adr_fx;             /* carrier phase relative to the IF since START, cycles * 2^32 (exact) */
    int64_t hist_adr[RX_DOP_DUMPS];  /* adr_fx and t_samp at the last RX_DOP_DUMPS epochs (Doppler) */
    uint64_t hist_t[RX_DOP_DUMPS];
    int hist_head, hist_n;
    int32_t cur_carr;           /* words in force since last_t */
    uint64_t cur_code;
    int32_t sent_carr;          /* the last command sent */
    uint64_t sent_code;
    uint32_t pend_seq[4];       /* commands sent and not yet applied: their (extended) tags and words, oldest first */
    int32_t pend_carr[4];
    uint64_t pend_code[4];
    int npend;
    int have_dump;
    double los[3];              /* unit vector receiver -> satellite from the last fix (aiding) */
    int have_los;
    /* Carrier smoothing (Hatch filter): the smoothed pseudorange, the carrier phase (m) and
     * sample it was last updated at, samples in it, the half-cycle state it assumed, and the
     * sample it last restarted at (its age sets the pseudorange's weight in the fix). */
    double hatch_pr, hatch_adr;
    uint64_t hatch_t, hatch_t0;
    uint32_t hatch_n;
    int hatch_inv;
    float res_var;              /* its squared normalized pseudorange residual, low-passed (0: none yet) */
    uint64_t res_t;             /* and the sample it was last updated at */
    /* Pilot channels (aided starts, rx_aid), and GPS channels timed from the seed (ms_valid):
     * code period, the week's count of code periods at the epoch that opened period 1, and the
     * secondary code, one chip per period. */
    double t_code;
    int64_t n1;
    int ms_valid;               /* GPS: n1 resolved from the seed or a fix (not the navigation message) */
    uint16_t sec_len;
    uint8_t sec[BDS_B1C_SEC_LEN];
    /* The integrity gate (GPS): whether the channel's code phase and Doppler agree with the
     * others' and the seed's or fix's prediction, how long it has failed, and whether its own
     * navigation message has confirmed its millisecond. */
    uint8_t gate_ok;
    uint8_t nav_ok;
    float gate_bad_s;
} rx_nco_t;

typedef struct {
    rx_cfg_t cfg;
    int32_t if_word;
    uint64_t code_word0;
    float carr_k, code_k;

    trk_ch_t ch[CORR_MAX_CH];
    rx_nco_t nco[CORR_MAX_CH];
    lnav_t nav[CORR_MAX_CH];
    int sat_ch[GNSS_SYS_COUNT][GNSS_MAX_PRN + 1];  /* channel tracking each satellite, -1 none */

    gps_eph_t eph[GNSS_SYS_COUNT][GNSS_MAX_PRN + 1];  /* GPS decoded; others preloaded (rx_aid) */
    gps_iono_t iono;
    int week;

    uint64_t next_acq;              /* sample count of the next search */
    uint64_t next_aid;              /* and of the next aided start */
    uint64_t aid_hold[GNSS_SYS_COUNT][GNSS_MAX_PRN + 1];  /* no aided start for this satellite before */
    int week_ref;                   /* rx_set_week_ref */
    int acc_valid;                  /* IMU aiding: the vehicle's acceleration (rx_set_accel) */
    double acc[3];
    int clk_rate_valid;             /* and the reference oscillator's predicted rate (rx_set_clock_rate) */
    double clk_rate;

    /* Receiver time: GPS time (s of week) of sample clk_n is clk_t. */
    int clk_valid;
    double clk_t;
    uint64_t clk_n;
    pvt_sol_t sol;
    /* The flight computer's prior (rx_set_seed): a position and time, held as a solution-shaped
     * state for prediction until the first fix. */
    int seed_valid;
    pvt_sol_t seed;
    double seed_pos_sigma, seed_tow_sigma;
    /* Coarse time: the seed's time was not good to a fraction of a millisecond, so the receiver
     * time and every seed-resolved transmit time share an unknown whole-millisecond offset. Fixes
     * solve it (pvt_opt_t.coarse_time) until two satellites' navigation messages agree on it;
     * then everything moves by it together (anchor_ms, at sample t_anchor). */
    int time_coarse;
    int64_t anchor_ms;
    uint64_t t_anchor;
    int retimed;                    /* the time moved: pilots started on the old one stop, holds clear */
    uint32_t n_ms_fixed;            /* GPS channels whose millisecond the seed or a fix resolved */
    uint32_t n_nav_reset;           /* channels whose navigation-message time the rest contradicted */
    /* The integrity gate: the seed's velocity uncertainty (rx_set_seed_vel; 1 km/s until told),
     * the platform's interference flag (rx_set_interference), and what the gate did. */
    double seed_vel_sigma;
    int interference;
    uint64_t t_gate, t_sol;         /* the gate's last check; the sample of the last fix */
    uint32_t n_gate_drop;           /* channels dropped: below the horizon, or out of agreement */
    uint32_t n_gate_withheld;       /* fixes withheld for want of redundancy */
    pvt_opt_t pvt_opt;
    int boost;                      /* the boost profile is the target */
    trk_profile_t prof;             /* the loops in force, moving toward the target (trk_profile_step) */
    uint64_t t_tick;                /* the last tick's sample */
} rx_t;

void rx_default_cfg(rx_cfg_t *c, double fs, double if_hz);
void rx_init(rx_t *rx, const rx_cfg_t *cfg);

/* Selects the boost loop profile (on) or the quiet one; the flight computer's phase decides. */
void rx_set_boost(rx_t *rx, int on);

/*
 * IMU aiding: the vehicle's acceleration (kinematic, ECEF, m/s^2: the IMU's specific force
 * rotated by the attitude, plus gravity) for the ticks that follow; valid = 0 stops aiding.
 * Each channel's loops get the line-of-sight Doppler rate it predicts, a.u / lambda, as
 * feed-forward, so they track only what the IMU misses and can stay narrow through a boost.
 */
void rx_set_accel(rx_t *rx, const double acc_ecef[3], int valid);

/*
 * The reference oscillator's predicted frequency rate, Hz/s at L1 (valid = 0 stops it): its
 * g-sensitivity times the rate of the specific force the IMU measures, -f_L1 Gamma.(df/dt).
 * Every channel shares the oscillator, so every channel gets it as feed-forward, with the line
 * of sight's own from rx_set_accel. The words stay on the same IF, so the observables keep the
 * clock's true drift and the fix's drift state takes it.
 */
void rx_set_clock_rate(rx_t *rx, double hz_per_s, int valid);

/* A full GPS week near today's (the flight computer's clock, a file's date): the navigation
 * message's 10-bit week resolves to the nearest. Default LNAV_WEEK_REF (2019-2038). */
void rx_set_week_ref(rx_t *rx, int week);

/*
 * The flight computer's prior: an ECEF position (m), and the GPS week and time of week (s) at
 * sample t, with how far each may be off (1-sigma, m and s). With it the receiver:
 *   - times each GPS satellite from its code phase alone, resolving the millisecond the code
 *     leaves open from the predicted range, so it fixes without waiting for the navigation
 *     message (the position must be good to tens of km);
 *   - when the time is not good to a fraction of a millisecond, carries its whole-millisecond
 *     error as one more unknown in the fix (coarse time, five satellites or more) until the
 *     navigation messages settle it;
 *   - aids the loops from launch, the lines of sight coming from the seed until the first fix;
 *   - checks each satellite's navigation-message time against the rest, and restarts the bit
 *     sync of one that is whole milliseconds out.
 */
void rx_set_seed(rx_t *rx, const double pos_ecef[3], double pos_sigma_m, int week, double tow,
                 double tow_sigma_s, uint64_t t);

/* The seed's velocity (ECEF, m/s) and its 1-sigma: it narrows the gate's Doppler window before
 * the first fix (on the pad, zero and about 1 m/s). Call after rx_set_seed. */
void rx_set_seed_vel(rx_t *rx, const double vel_ecef[3], double vel_sigma_mps);

/*
 * The platform's interference flag (the FPGA stage's input-to-output power, say): while it is
 * set, every fix needs the redundancy the gate otherwise asks only of unconfirmed ranges, and
 * fixes carry the flag.
 *
 * The integrity gate itself needs no call. A tone near L1 leaks through the C/A code's spectral
 * lines into channels that track nothing real, and a seed's millisecond would turn them into
 * ranges. So, with a seed or a fix:
 *   - satellites predicted below -5 deg are not searched, and channels on them are dropped;
 *   - a channel gets its millisecond from the seed only in a group of three or more whose code
 *     phases and Dopplers agree, within windows from the seed's position and velocity sigmas;
 *     after a fix, only if its own agree with the fix's prediction (5 us, 100 Hz). One that
 *     fails for 2 s is dropped, and its PRN rests 10 s;
 *   - a fix using ranges no navigation message has confirmed needs a spare degree of freedom,
 *     and a velocity that passes its own test with one; otherwise it is withheld.
 */
void rx_set_interference(rx_t *rx, int flag);

/* The 1 ms tick at sample t_now. Returns the number of commands written. */
int rx_tick(rx_t *rx, uint64_t t_now, const corr_dump_t *d, int nd, corr_cmd_t *cmds, int ncap);

/* Whether a snapshot is wanted at t_now; *ms receives its length. */
int rx_wants_snapshot(const rx_t *rx, uint64_t t_now, int *ms);

/*
 * Searches the PRNs not in a channel in a snapshot whose first sample is t0,
 * and starts channels for detections (commands for sample t_now onward).
 * work: acq_work_floats() floats. Returns the number of commands written.
 */
int rx_acquire(rx_t *rx, uint64_t t_now, uint64_t t0, const float *iq, size_t n, float *work, corr_cmd_t *cmds,
               int ncap);

/*
 * Aided starts (milestone 6): once GPS has a fix, starts a channel on the pilot of every
 * Galileo and BeiDou satellite with a preloaded ephemeris (rx->eph) above 10 degrees, at the
 * code phase and Doppler the fix predicts for sample t_now + 1 ms: E1-C with E1-B beside it,
 * or the B1C pilot with its data. Acts at most once a second. Returns the commands written.
 */
int rx_aid(rx_t *rx, uint64_t t_now, corr_cmd_t *cmds, int ncap);

/* Observables and PVT at sample t (the latest tick). Returns the number of observables. */
int rx_measure(rx_t *rx, uint64_t t, rx_obs_t *obs, int max, pvt_sol_t *sol);

/* GPS time (s of week) of sample t, once the clock is set. */
double rx_time(const rx_t *rx, uint64_t t);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_RX_H */

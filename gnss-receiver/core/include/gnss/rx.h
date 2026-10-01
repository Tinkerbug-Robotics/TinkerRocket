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
} rx_cfg_t;

/* Per-satellite record of an epoch (what goes to RINEX and the logs). */
typedef struct {
    int prn;
    int ch;
    double pr;                  /* m */
    double adr;                 /* carrier phase, cycles, RINEX sign (grows with range) */
    double dop;                 /* Hz */
    double t_sv;                /* transmit time, s of week (satellite clock) */
    float cn0;                  /* dB-Hz */
    float lock_s;               /* time since the PLL locked */
    int half_cycle;             /* the Costas half-cycle ambiguity is resolved */
} rx_obs_t;

typedef struct {
    uint64_t t_start;           /* sample the channel started at */
    uint64_t last_t;            /* t_samp of the last dump */
    uint64_t last_code_phase;
    uint32_t last_carr_phase;
    uint32_t last_carr_cycles;
    int64_t adr_fx;             /* carrier phase relative to the IF since START, cycles * 2^32 (exact) */
    int32_t cur_carr;           /* words in force since last_t */
    uint64_t cur_code;
    int32_t sent_carr;          /* the last command sent, and the tick it was sent on */
    uint64_t sent_code;
    uint64_t sent_t;
    int have_dump;
} rx_nco_t;

typedef struct {
    rx_cfg_t cfg;
    int32_t if_word;
    uint64_t code_word0;
    float carr_k, code_k;

    trk_ch_t ch[CORR_MAX_CH];
    rx_nco_t nco[CORR_MAX_CH];
    lnav_t nav[CORR_MAX_CH];
    int prn_ch[GPS_MAX_PRN + 1];    /* channel tracking each PRN, -1 none */

    gps_eph_t eph[GPS_MAX_PRN + 1];
    gps_iono_t iono;
    int week;

    uint64_t next_acq;              /* sample count of the next search */

    /* Receiver time: GPS time (s of week) of sample clk_n is clk_t. */
    int clk_valid;
    double clk_t;
    uint64_t clk_n;
    pvt_sol_t sol;
    pvt_opt_t pvt_opt;
} rx_t;

void rx_default_cfg(rx_cfg_t *c, double fs, double if_hz);
void rx_init(rx_t *rx, const rx_cfg_t *cfg);

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

/* Observables and PVT at sample t (the latest tick). Returns the number of observables. */
int rx_measure(rx_t *rx, uint64_t t, rx_obs_t *obs, int max, pvt_sol_t *sol);

/* GPS time (s of week) of sample t, once the clock is set. */
double rx_time(const rx_t *rx, uint64_t t);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_RX_H */

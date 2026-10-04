/*
 * The board's IMU as the P4 reads it for aiding: an ST ISM6HG256X (DS15034 Rev 2), the part on Mantis and on the
 * dev board, mounted at 45 deg in the board plane on both. Only its accelerometer along the thrust axis is modelled:
 * these flights go straight up, and the attitude error is a separate knob (gnssrx --imu-ism6 TILT).
 *
 * Two sensor axes share the thrust (at the mount angle and 90 deg from it). Each is sampled at the output data rate
 * through LPF1 (cutoff ODR/2; about one sample of group delay), reaches the P4 a transport delay later, and is held
 * until the next sample. Each axis reads the low-g channel (+-16 g) until it nears that rail, then the high-g one
 * (+-32 to +-256 g), and each channel has its own noise density, zero-g offset, sensitivity error, LSB and range;
 * the high-g channel also its nonlinearity (2 %FS at 256 g; taken as quadratic, the same fraction at any range).
 *
 * Pad calibration: at rest on the pad the thrust axis reads 1 g, so the P4 takes each channel's offset on each axis
 * there, the sensitivity error's share at 1 g with it. What is left in flight is the sensitivity error on the load
 * above 1 g and the nonlinearity's change.
 *
 * Datasheet values (table 3), per sensor axis:
 *                      low-g (+-16 g)     high-g (+-32..256 g)
 *   noise density      65 ug/rtHz (100 max)  1000 ug/rtHz (1100 max)
 *   zero-g offset      +-10 mg (65 max)   +-250 mg (1000 max)
 *   sensitivity        +-1 % (3 sigma)    +-1 % (3 sigma)
 *   LSB                0.488 mg           0.976, 1.952, 3.904, 10.417 mg at 32, 64, 128, 256 g
 *   nonlinearity       not stated         2 %FS at 256 g (typ)
 *   ODR                up to 7.68 kHz     480 Hz to 7.68 kHz
 */
#ifndef GNSS_HOST_IMU_ISM6_H
#define GNSS_HOST_IMU_ISM6_H

#include <stdint.h>

#include "rng.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double odr_hz;            /* output data rate: 480, 960, 1920, 3840 or 7680 (the high-g channel's minimum is 480) */
    double hg_fs_g;           /* high-g full scale: 32, 64, 128 or 256 */
    double mount_deg;         /* sensor axis A from the thrust axis; B is 90 deg from A (45: Mantis, the dev board) */
    int pad_cal;              /* the P4 takes each channel's offset at rest on the pad */
    double off_lg_g, off_hg_g;  /* zero-g offsets, g (the same sign on both axes: the worst for their sum) */
    double sf_lg, sf_hg;      /* sensitivity errors, fraction (likewise) */
    double nl_hg;             /* high-g nonlinearity, fraction of its full scale */
    double nd_lg, nd_hg;      /* noise densities, g/sqrt(Hz) */
    double transport_s;       /* from the end of the sample to the P4 having it */
} ism6_cfg_t;

/* The datasheet's typical part (offsets at their typical values, sensitivity a third of its 3-sigma tolerance) or
 * its worst (the maxima), 960 Hz, +-64 g high-g, at 45 deg, pad-calibrated, 0.2 ms transport. */
void ism6_cfg_default(ism6_cfg_t *c, int worst);

typedef struct {
    ism6_cfg_t cfg;
    rng_t rng;
    double t_next;            /* the next sample's time, s */
    double held;              /* the P4's axial specific force from the latest sample, m/s^2 */
    double cal_lg[2], cal_hg[2];  /* the P4's offset estimates, g */
    uint64_t n_samples, n_high, n_rail;  /* samples, axis-samples on the high-g channel, axis-samples at a rail */
} ism6_t;

/* t0: the first sample's time; the pad's axial specific force is 1 g. */
void ism6_init(ism6_t *m, const ism6_cfg_t *c, double t0, uint64_t seed);

/* The specific force along the thrust axis at time t, m/s^2 (+9.80665 at rest on the pad). */
typedef double (*ism6_force_fn)(void *ctx, double t);

/* Takes every sample the P4 has by time t and returns its axial specific force, m/s^2. */
double ism6_read(ism6_t *m, double t, ism6_force_fn f, void *ctx);

/* The group delay the P4's estimate lags the force by, on average: LPF1, transport, half a sample held. */
double ism6_mean_delay(const ism6_cfg_t *c);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_IMU_ISM6_H */

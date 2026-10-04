#include "imu_ism6.h"

#include <math.h>

#define ISM6_G 9.80665
#define ISM6_PI 3.14159265358979323846
#define ISM6_LG_FS 16.0
#define ISM6_LG_LSB 0.488e-3
#define ISM6_SWITCH 0.97        /* the P4 moves an axis to the high-g channel at 97 % of the low-g rail */
#define ISM6_CAL_S 5.0          /* the pad calibration averages this long */

void ism6_cfg_default(ism6_cfg_t *c, int worst)
{
    c->odr_hz = 960.0;
    c->hg_fs_g = 64.0;
    c->mount_deg = 45.0;
    c->pad_cal = 1;
    c->off_lg_g = worst ? 65e-3 : 10e-3;
    c->off_hg_g = worst ? 1000e-3 : 250e-3;
    c->sf_lg = worst ? 0.01 : 0.01 / 3.0;
    c->sf_hg = worst ? 0.01 : 0.01 / 3.0;
    c->nl_hg = 0.02;
    c->nd_lg = worst ? 100e-6 : 65e-6;
    c->nd_hg = worst ? 1100e-6 : 1000e-6;
    c->transport_s = 0.2e-3;
}

static double hg_lsb(double fs)
{
    return fs <= 32.0 ? 0.976e-3 : fs <= 64.0 ? 1.952e-3 : fs <= 128.0 ? 3.904e-3 : 10.417e-3;
}

/* The high-g channel's deviation from its line at f g: the stated 2 %FS, reached at full scale. */
static double hg_nl(const ism6_cfg_t *c, double f)
{
    const double x = f / c->hg_fs_g;
    return c->nl_hg * c->hg_fs_g * x * x;
}

static double quantize(double v, double lsb, double fs)
{
    v = v > fs ? fs : v < -fs ? -fs : v;
    return lsb * floor(v / lsb + 0.5);
}

void ism6_init(ism6_t *m, const ism6_cfg_t *c, double t0, uint64_t seed)
{
    m->cfg = *c;
    rng_seed(&m->rng, seed);
    m->t_next = t0;
    m->held = ISM6_G;
    m->n_samples = m->n_high = m->n_rail = 0;
    /* The pad: each axis sees 1 g times its cosine to the thrust. The P4's estimate of each channel's offset is
     * what it reads there less that, averaged over ISM6_CAL_S. */
    const double th = c->mount_deg * ISM6_PI / 180.0;
    const double cs[2] = {cos(th), sin(th)};
    const double n_avg = c->odr_hz * ISM6_CAL_S;
    for (int i = 0; i < 2; i++) {
        double z0, z1;
        rng_gauss2(&m->rng, &z0, &z1);
        const double f = cs[i];
        m->cal_lg[i] = c->pad_cal ? c->off_lg_g + c->sf_lg * f + z0 * c->nd_lg * sqrt(0.5 * c->odr_hz / n_avg) : 0.0;
        m->cal_hg[i] = c->pad_cal ? c->off_hg_g + c->sf_hg * f + hg_nl(c, f) + z1 * c->nd_hg * sqrt(0.5 * c->odr_hz / n_avg)
                                  : 0.0;
    }
}

double ism6_mean_delay(const ism6_cfg_t *c)
{
    return 1.0 / c->odr_hz + c->transport_s + 0.5 / c->odr_hz;
}

double ism6_read(ism6_t *m, double t, ism6_force_fn force, void *ctx)
{
    const ism6_cfg_t *c = &m->cfg;
    const double dt = 1.0 / c->odr_hz;
    const double th = c->mount_deg * ISM6_PI / 180.0;
    const double cs[2] = {cos(th), sin(th)};
    const double sg_lg = c->nd_lg * sqrt(0.5 * c->odr_hz), sg_hg = c->nd_hg * sqrt(0.5 * c->odr_hz);
    while (t >= m->t_next + c->transport_s) {
        /* LPF1's group delay, about one sample at ODR/2. */
        const double f_g = force(ctx, m->t_next - dt) / ISM6_G;
        double z[2], est = 0.0;
        rng_gauss2(&m->rng, &z[0], &z[1]);
        for (int i = 0; i < 2; i++) {
            const double f = f_g * cs[i];
            const double lg = quantize(f * (1.0 + c->sf_lg) + c->off_lg_g + z[i] * sg_lg, ISM6_LG_LSB, ISM6_LG_FS);
            double r;
            if (fabs(lg) < ISM6_SWITCH * ISM6_LG_FS) {
                r = lg - m->cal_lg[i];
            } else {
                const double hg = f * (1.0 + c->sf_hg) + hg_nl(c, f) + c->off_hg_g + z[i] * sg_hg;
                const double q = quantize(hg, hg_lsb(c->hg_fs_g), c->hg_fs_g);
                m->n_rail += fabs(q) >= c->hg_fs_g;
                m->n_high++;
                r = q - m->cal_hg[i];
            }
            est += r * cs[i];
        }
        m->held = est * ISM6_G;
        m->n_samples++;
        m->t_next += dt;
    }
    return m->held;
}

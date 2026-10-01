/*
 * Position, velocity and time by least squares, in double (P4: software
 * double, run only at the PVT rate).
 */
#ifndef GNSS_PVT_H
#define GNSS_PVT_H

#include "gnss/eph.h"
#include "gnss/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PVT_MAX_SAT 32

/* One satellite's measurements at an epoch. */
typedef struct {
    int sys;          /* gnss_sys_t */
    int prn;
    double pr;        /* pseudorange, m: GPS receive time less the transmit time in GPS time */
    double dop;       /* Doppler, Hz (positive approaching) */
    double t_sv;      /* transmit time on the satellite's clock, s of week in its own system's time */
    float cn0;        /* dB-Hz */
    float sigma;      /* pseudorange 1-sigma, m (0: pvt_sigma_pr from cn0, unsmoothed) */
    float sigma_dop;  /* range-rate 1-sigma, m/s (0: pvt_sigma_dop from cn0, carrier locked, 10 Hz) */
} pvt_meas_t;

typedef struct {
    int valid;
    int nsat;                 /* satellites used */
    double pos[3];            /* ECEF, m */
    double vel[3];            /* ECEF, m/s */
    double lat, lon, h;       /* rad, rad, m (WGS 84 ellipsoid) */
    double clk_bias;          /* receiver clock against GPS time, m (positive: the receiver clock is ahead) */
    double isb[GNSS_SYS_COUNT];  /* each other system's time against GPS's, as solved, m (0 if absent) */
    double clk_drift;         /* m/s */
    double pdop, hdop, vdop;
    double resid_rms;         /* m */
    double resid[PVT_MAX_SAT];
    double el[PVT_MAX_SAT], az[PVT_MAX_SAT];  /* rad, per input measurement */
    double los[PVT_MAX_SAT][3];               /* unit vector receiver -> satellite, ECEF */
    int used[PVT_MAX_SAT];
    int iter;
    /* The residual test: measurements it left out (bit 0 the pseudorange, bit 1 the Doppler), how
     * many, the position's weighted residual sum against its line, and whether the velocity
     * passed (its Dopplers can fail where the ranges pass). */
    int excluded[PVT_MAX_SAT];
    int nexcl;
    double chi2, chi2_lim;
    int vel_valid;
} pvt_sol_t;

typedef struct {
    double el_mask;           /* rad */
    int use_iono, use_tropo;
    /* Weighted least squares, then a chi-square test on the weighted residuals: on failure the
     * worst measurement is left out and the solve repeated, at most raim_max_excl times; a fix
     * that still fails is withheld (pvt_solve returns -2). raim = 0 skips the test. */
    int raim;
    double raim_pfa;          /* false-alarm probability per epoch */
    int raim_max_excl;
} pvt_opt_t;

/*
 * Measurement 1-sigmas for the weights and the test. pvt_sigma_pr: a pseudorange at cn0
 * (dB-Hz), carrier-smoothed for smooth_s seconds (0: raw). pvt_sigma_dop: a range rate, from
 * a carrier-locked channel or (locked = 0) one whose PLL is pulling in, behind a PLL of
 * pll_bw Hz (0: 10 Hz).
 */
double pvt_sigma_pr(float cn0, float smooth_s);
double pvt_sigma_dop(float cn0, int locked, float pll_bw);

void pvt_default_opt(pvt_opt_t *o);

/*
 * Solves for position, clock, velocity and clock drift, plus one time offset per system
 * other than GPS that has measurements. eph is indexed [sys][prn]; iono may be NULL. pos0
 * seeds the iteration (NULL: Earth's centre). Returns 0 on success, -2 when the fix fails the
 * residual test (sol->valid stays 0), -1 when there is none.
 */
int pvt_solve(const pvt_meas_t *m, int n, const gps_eph_t (*eph)[GNSS_MAX_PRN + 1], const gps_iono_t *iono,
              const pvt_opt_t *opt, const double *pos0, pvt_sol_t *sol);

/* WGS 84 ECEF <-> geodetic (rad, rad, m). */
void ecef_to_geo(const double x[3], double *lat, double *lon, double *h);
void geo_to_ecef(double lat, double lon, double h, double x[3]);

/* Klobuchar delay on L1 (m) for a receiver at lat/lon (rad), satellite at az/el (rad), GPS time t (s of week). */
double iono_klobuchar(const gps_iono_t *io, double lat, double lon, double az, double el, double t);

/* Saastamoinen delay (m) with a standard atmosphere, for height h (m, up to 40 km) and elevation el (rad). */
double tropo_saastamoinen(double lat, double h, double el);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_PVT_H */

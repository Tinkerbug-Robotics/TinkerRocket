/*
 * Position, velocity and time by least squares, in double (P4: software
 * double, run only at the PVT rate).
 */
#ifndef GNSS_PVT_H
#define GNSS_PVT_H

#include "gnss/eph.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PVT_MAX_SAT 32

/* One satellite's measurements at an epoch. */
typedef struct {
    int prn;
    double pr;        /* pseudorange, m */
    double dop;       /* Doppler, Hz (positive approaching) */
    double t_sv;      /* transmit time on the satellite's clock, s of week */
    float cn0;        /* dB-Hz */
} pvt_meas_t;

typedef struct {
    int valid;
    int nsat;                 /* satellites used */
    double pos[3];            /* ECEF, m */
    double vel[3];            /* ECEF, m/s */
    double lat, lon, h;       /* rad, rad, m (WGS 84 ellipsoid) */
    double clk_bias;          /* receiver clock, m (positive: the receiver clock is ahead) */
    double clk_drift;         /* m/s */
    double pdop, hdop, vdop;
    double resid_rms;         /* m */
    double resid[PVT_MAX_SAT];
    double el[PVT_MAX_SAT], az[PVT_MAX_SAT];  /* rad, per input measurement */
    int used[PVT_MAX_SAT];
    int iter;
} pvt_sol_t;

typedef struct {
    double el_mask;           /* rad */
    int use_iono, use_tropo;
} pvt_opt_t;

void pvt_default_opt(pvt_opt_t *o);

/*
 * Solves for position, clock, velocity and clock drift. eph is indexed by PRN
 * (eph[prn]); iono may be NULL. pos0 seeds the iteration (NULL: Earth's
 * centre). Returns 0 on success.
 */
int pvt_solve(const pvt_meas_t *m, int n, const gps_eph_t *eph, const gps_iono_t *iono, const pvt_opt_t *opt,
              const double *pos0, pvt_sol_t *sol);

/* WGS 84 ECEF <-> geodetic (rad, rad, m). */
void ecef_to_geo(const double x[3], double *lat, double *lon, double *h);
void geo_to_ecef(double lat, double lon, double h, double x[3]);

/* Klobuchar delay on L1 (m) for a receiver at lat/lon (rad), satellite at az/el (rad), GPS time t (s of week). */
double iono_klobuchar(const gps_iono_t *io, double lat, double lon, double az, double el, double t);

/* Saastamoinen delay (m) with a standard atmosphere, for height h (m) and elevation el (rad). */
double tropo_saastamoinen(double lat, double h, double el);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_PVT_H */

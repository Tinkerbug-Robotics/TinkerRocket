/* GPS broadcast ephemeris and ionosphere parameters, and satellite position from them (IS-GPS-200). */
#ifndef GNSS_EPH_H
#define GNSS_EPH_H

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_MU       3.986005e14         /* m^3/s^2, IS-GPS-200 */
#define GPS_OMEGA_E  7.2921151467e-5     /* rad/s */
#define GPS_PI_ICD   3.1415926535898     /* the value IS-GPS-200 tells receivers to use */
#define GPS_F_REL    -4.442807633e-10    /* s/m^0.5, relativistic clock term */

typedef struct {
    int valid;
    int prn;
    int week;            /* full GPS week of toe */
    int iodc, iode, health, ura;
    double toe, toc;     /* s of week */
    double sqrt_a, e, i0, omega0, omega, m0, delta_n, idot, omega_dot;  /* rad, rad/s */
    double cuc, cus, crc, crs, cic, cis;
    double af0, af1, af2, tgd;
} gps_eph_t;

typedef struct {
    int valid;
    double alpha[4], beta[4];
} gps_iono_t;

/*
 * Satellite position (ECEF at the transmit time, m) and clock correction (s,
 * including the relativistic term and minus TGD, so it applies to an L1 C/A
 * pseudorange) at GPS time t (s of week, the satellite's own time corrected for
 * its clock). vel may be NULL.
 */
void gps_sat_pos(const gps_eph_t *e, double t, double pos[3], double vel[3], double *clk);

/* Satellite clock correction at the satellite's own transmit time t_sv (s of week), before the
 * position is known: the relativistic term is included by an iteration. */
double gps_sat_clock(const gps_eph_t *e, double t_sv);

/* t1 - t0 wrapped into +-half a week. */
double gps_time_diff(double t1, double t0);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_EPH_H */

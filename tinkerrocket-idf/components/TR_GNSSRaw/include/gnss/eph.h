/* From the GNSS receiver (gnss repo receiver/core/include/gnss/eph.h at 7511d11), with GLONASS added here. */
/*
 * Broadcast ephemeris and ionosphere parameters, and satellite position from them: GPS
 * (IS-GPS-200), and the same Keplerian set for Galileo (OS SIS ICD) and BeiDou MEO/IGSO
 * (BDS-SIS-ICD), each with its own constants and in its own time (GST seconds of week track
 * GPS's; BDT = GPST - 14 s).
 *
 * GLONASS (sys GNSS_SYS_GLO, ICD L1/L2 FDMA 5.1) broadcasts no Keplerian set: its satellite's
 * state at tb in PZ-90.11 (position, velocity and the Moon's and Sun's acceleration), which the
 * position below integrates (fourth-order Runge-Kutta, 60 s steps, the Earth's J2 term; PZ-90.11
 * sits within centimetres of WGS 84). Its set rides in the same struct, in GPS time like the
 * others so a caller treats it as it does GPS: toe = toc = tb as GPS seconds of week (the
 * broadcast's UTC(SU) + 3 h moved by the leap seconds), week the GPS week of tb; af0 = -tau_n
 * and af1 = +gamma_n (RINEX's own signs: the clock is af0 + af1 (t - toc), relativity already in
 * it), af2 = tgd = 0; glo_pos, glo_vel, glo_acc the state; glo_k the frequency number. The
 * Keplerian fields are unused. iode is tb's index in the day (tb / 900 s), as RTCM 1020 and
 * RINEX file an issue.
 */
#ifndef GNSS_EPH_H
#define GNSS_EPH_H

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_MU       3.986005e14         /* m^3/s^2, IS-GPS-200 */
#define GPS_OMEGA_E  7.2921151467e-5     /* rad/s */
#define GPS_PI_ICD   3.1415926535898     /* the value IS-GPS-200 tells receivers to use */
#define GPS_F_REL    -4.442807633e-10    /* s/m^0.5, relativistic clock term */
#define GAL_MU       3.986004418e14      /* m^3/s^2, OS SIS ICD */
#define BDS_MU       3.986004418e14      /* CGCS2000 */
#define BDS_OMEGA_E  7.2921150e-5        /* rad/s */
#define GAL_F_REL    -4.442807309e-10
#define BDT_MINUS_GPST -14.0             /* s */
#define GLO_MU       3.9860044e14        /* m^3/s^2, PZ-90.11 */
#define GLO_OMEGA_E  7.292115e-5         /* rad/s */
#define GLO_AE       6378136.0           /* m, the equatorial radius */
#define GLO_J20      1.08262575e-3       /* the second zonal harmonic */
#define GLO_F1_HZ    1602.0e6            /* L1OF's centre, k = 0 */
#define GLO_DF1_HZ   562.5e3             /* its step per frequency number */
#define GLO_STEP_S   60.0                /* the integration's step */

#define GNSS_MAX_PRN 63                  /* the largest PRN of any system (BeiDou) */

typedef struct {
    int valid;
    int sys;             /* gnss_sys_t; 0 = GPS */
    int prn;
    int week;            /* full GPS week of toe */
    int iodc, iode, health, ura;
    double toe, toc;     /* s of week */
    double sqrt_a, e, i0, omega0, omega, m0, delta_n, idot, omega_dot;  /* rad, rad/s */
    double cuc, cus, crc, crs, cic, cis;
    double af0, af1, af2, tgd;  /* tgd: GPS TGD; Galileo BGD(E1,E5b); BeiDou the B1C group delay */
    /* GLONASS only (above): the state at tb, PZ-90.11, m, m/s, m/s^2, and the frequency number (-7..+6) */
    double glo_pos[3], glo_vel[3], glo_acc[3];
    int glo_k;
} gps_eph_t;

typedef struct {
    int valid;
    double alpha[4], beta[4];
} gps_iono_t;

/*
 * Satellite position (ECEF at the transmit time, m) and clock correction (s,
 * including the relativistic term and minus TGD, so it applies to an L1-band
 * pseudorange) at system time t (s of week in the ephemeris's own system, the
 * satellite's time corrected for its clock). vel may be NULL.
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

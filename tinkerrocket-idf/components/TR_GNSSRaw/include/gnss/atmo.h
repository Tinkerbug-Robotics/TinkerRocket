/*
 * The receiver-side corrections a raw pseudorange needs before a navigation filter can use it, and
 * the geodesy they are evaluated in: WGS 84 geodetic <-> ECEF, a satellite's azimuth and elevation,
 * the GPS broadcast (Klobuchar) ionosphere and the Saastamoinen troposphere. From the GNSS receiver
 * (gnss repo receiver/core/pvt/pvt.c at 7511d11, where they serve its least-squares fix), unchanged
 * but for az_el, made public here as gnss_az_el. Double precision throughout: they run once per
 * satellite per raw epoch (1 Hz on the LC86G), where software double on the S3 costs little.
 */
#ifndef GNSS_ATMO_H
#define GNSS_ATMO_H
#include "gnss/eph.h"
#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_C 299792458.0          /* speed of light, m/s (IS-GPS-200) */

/* Geodetic latitude, longitude (rad) and ellipsoidal height (m) <-> ECEF (m), WGS 84. */
void geo_to_ecef(double lat, double lon, double h, double x[3]);
void ecef_to_geo(const double x[3], double *lat, double *lon, double *h);

/* Azimuth (rad, from north through east) and elevation (rad) of the direction d (ECEF, any
 * length) seen from geodetic lat, lon. */
void gnss_az_el(double lat, double lon, const double d[3], double *az, double *el);

/*
 * Ionospheric group delay on GPS L1 (m) from the broadcast Klobuchar parameters (IS-GPS-200
 * 20.3.3.5.2.5), at the receiver's geodetic lat/lon (rad), the satellite's azimuth/elevation (rad),
 * and GPS time of day t (s; the model takes it modulo a day). Another frequency f scales it by
 * (1575.42 MHz / f)^2. With no parameters (all zero) only the model's night-time 5 ns is left.
 */
double iono_klobuchar(const gps_iono_t *io, double lat, double lon, double az, double el, double t);

/*
 * Tropospheric delay (m) for a receiver at geodetic latitude lat (rad) and ellipsoidal height h (m),
 * to a satellite at elevation el (rad): Saastamoinen in the standard atmosphere at 70 % humidity,
 * mapped by 1/sin(el), with the stratosphere above 11 km and nothing above 40 km.
 */
double tropo_saastamoinen(double lat, double h, double el);

#ifdef __cplusplus
}
#endif
#endif /* GNSS_ATMO_H */

/* From the GNSS receiver (gnss repo receiver/core/pvt/pvt.c at 7511d11), unchanged but for az_el,
 * made public as gnss_az_el. See gnss/atmo.h. */
#include "gnss/atmo.h"

#include <math.h>

#define WGS84_A  6378137.0
#define WGS84_F  (1.0 / 298.257223563)
#define WGS84_E2 (WGS84_F * (2.0 - WGS84_F))
#define PI       3.14159265358979323846

void geo_to_ecef(double lat, double lon, double h, double x[3])
{
    double s = sin(lat), c = cos(lat);
    double n = WGS84_A / sqrt(1.0 - WGS84_E2 * s * s);
    x[0] = (n + h) * c * cos(lon);
    x[1] = (n + h) * c * sin(lon);
    x[2] = (n * (1.0 - WGS84_E2) + h) * s;
}

void ecef_to_geo(const double x[3], double *lat, double *lon, double *h)
{
    double p = sqrt(x[0] * x[0] + x[1] * x[1]);
    double la = atan2(x[2], p * (1.0 - WGS84_E2));
    double n = WGS84_A;
    for (int i = 0; i < 10; i++) {
        double s = sin(la);
        n = WGS84_A / sqrt(1.0 - WGS84_E2 * s * s);
        double hh = p / cos(la) - n;
        if (p < 1e-3) {
            hh = fabs(x[2]) - WGS84_A * sqrt(1.0 - WGS84_E2);
        }
        la = atan2(x[2], p * (1.0 - WGS84_E2 * n / (n + hh)));
        *h = hh;
    }
    *lat = la;
    *lon = atan2(x[1], x[0]);
    double s = sin(la);
    n = WGS84_A / sqrt(1.0 - WGS84_E2 * s * s);
    *h = (p > 1e-3) ? p / cos(la) - n : fabs(x[2]) - WGS84_A * sqrt(1.0 - WGS84_E2);
}

/* Unit vector rx -> sat in local east/north/up, giving azimuth and elevation. */
void gnss_az_el(double lat, double lon, const double d[3], double *az, double *el)
{
    double sl = sin(lat), cl = cos(lat), so = sin(lon), co = cos(lon);
    double e = -so * d[0] + co * d[1];
    double n = -sl * co * d[0] - sl * so * d[1] + cl * d[2];
    double u = cl * co * d[0] + cl * so * d[1] + sl * d[2];
    *az = atan2(e, n);
    *el = asin(u / sqrt(e * e + n * n + u * u));
}

double iono_klobuchar(const gps_iono_t *io, double lat, double lon, double az, double el, double t)
{
    /* IS-GPS-200 20.3.3.5.2.5; angles in semicircles. */
    double phi_u = lat / PI, lam_u = lon / PI, e = el / PI;
    double psi = 0.0137 / (e + 0.11) - 0.022;
    double phi_i = phi_u + psi * cos(az);
    if (phi_i > 0.416) {
        phi_i = 0.416;
    } else if (phi_i < -0.416) {
        phi_i = -0.416;
    }
    double lam_i = lam_u + psi * sin(az) / cos(phi_i * PI);
    double phi_m = phi_i + 0.064 * cos((lam_i - 1.617) * PI);
    double tl = fmod(4.32e4 * lam_i + t, 86400.0);
    if (tl < 0.0) {
        tl += 86400.0;
    }
    double f = 1.0 + 16.0 * pow(0.53 - e, 3.0);
    double amp = io->alpha[0] + phi_m * (io->alpha[1] + phi_m * (io->alpha[2] + phi_m * io->alpha[3]));
    double per = io->beta[0] + phi_m * (io->beta[1] + phi_m * (io->beta[2] + phi_m * io->beta[3]));
    if (amp < 0.0) {
        amp = 0.0;
    }
    if (per < 72000.0) {
        per = 72000.0;
    }
    double x = 2.0 * PI * (tl - 50400.0) / per;
    double delay = (fabs(x) < 1.57) ? f * (5e-9 + amp * (1.0 - x * x / 2.0 + x * x * x * x / 24.0)) : f * 5e-9;
    return GNSS_C * delay;
}

double tropo_saastamoinen(double lat, double h, double el)
{
    /* No ceiling at 10 km, where the delay is still 0.6 m at the zenith: a rocket climbs through
     * the rest of it. The troposphere's pressure law carried on upward stays within a few cm of the
     * standard atmosphere's stratosphere, and reaches nothing by 40 km. */
    if (h < -100.0 || h > 4e4 || el <= 0.0) {
        return 0.0;
    }
    double hh = h < 0.0 ? 0.0 : h;
    /* Standard atmosphere, 70 % relative humidity. */
    double p = 1013.25 * pow(1.0 - 2.2557e-5 * hh, 5.2568);
    double tk = 15.0 - 6.5e-3 * hh + 273.16;
    /* Above the tropopause (11 km) the water holds the tropopause's temperature and thins with the
     * pressure. The lapse rate carried on up would reach 38 K at 38.4 km, where the vapour pressure's
     * formula divides by zero: the delay was infinite from there to 40 km. */
    double tw = tk, pw = 1.0;
    if (hh > 11000.0) {
        tw = 15.0 - 6.5e-3 * 11000.0 + 273.16;
        pw = p / (1013.25 * pow(1.0 - 2.2557e-5 * 11000.0, 5.2568));
    }
    double e = 6.108 * 0.7 * exp((17.15 * tw - 4684.0) / (tw - 38.45)) * pw;
    double z = PI / 2.0 - el;
    double dry = 0.0022768 * p / (1.0 - 0.00266 * cos(2.0 * lat) - 0.00028 * hh / 1e3) / cos(z);
    double wet = 0.002277 * (1255.0 / tw + 0.05) * e / cos(z);
    return dry + wet;
}

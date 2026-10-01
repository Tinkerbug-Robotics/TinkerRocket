/*
 * Math for the loops. Our own atan/atan2 so the host and the P4 compute the
 * same float32 results bit for bit (libm differs between platforms); with
 * -ffp-contract=off and no fast-math, +, -, *, / and sqrt round identically on
 * both.
 */
#ifndef GNSS_GMATH_H
#define GNSS_GMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_PI_F  3.14159265358979323846f
#define GNSS_PI    3.14159265358979323846

/* atan2(y, x) in radians, |error| < 3e-7. Returns 0 for (0, 0). */
float gnss_atan2f(float y, float x);

/* atan(y / x) folded to (-pi/2, pi/2]: insensitive to a sign flip of both (data bits). */
float gnss_atan_halff(float y, float x);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_GMATH_H */

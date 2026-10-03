#include "gnss/gmath.h"

/*
 * atan on [0, 1] as z * P(z^2), P of degree 8: a near-minimax fit (iteratively
 * reweighted least squares) to atan(z)/z, made for this file with numpy. In
 * float32 Horner form the worst error is 1.0e-7 rad.
 */
static float atan01(float z)
{
    float s = z * z;
    float p = 2.4565614294e-03f;
    p = p * s - 1.4400647022e-02f;
    p = p * s + 3.9779946208e-02f;
    p = p * s - 7.2347350419e-02f;
    p = p * s + 1.0498879850e-01f;
    p = p * s - 1.4161209762e-01f;
    p = p * s + 1.9985903800e-01f;
    p = p * s - 3.3332598209e-01f;
    p = p * s + 9.9999988079e-01f;
    return z * p;
}

float gnss_atan2f(float y, float x)
{
    float ax = x < 0.0f ? -x : x;
    float ay = y < 0.0f ? -y : y;
    if (ax == 0.0f && ay == 0.0f) {
        return 0.0f;
    }
    float a = (ay <= ax) ? atan01(ay / ax) : 0.5f * GNSS_PI_F - atan01(ax / ay);
    if (x < 0.0f) {
        a = GNSS_PI_F - a;
    }
    return y < 0.0f ? -a : a;
}

float gnss_atan_halff(float y, float x)
{
    if (x < 0.0f) {
        x = -x;
        y = -y;
    }
    return gnss_atan2f(y, x);
}

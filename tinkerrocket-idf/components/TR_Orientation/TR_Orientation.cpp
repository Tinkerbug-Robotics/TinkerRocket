#include "TR_Orientation.h"

#include <math.h>

// clock=0 base rotations: shortest arc taking the selected board axis to
// the rocket nose (+X).  Row-major, v_rocket = R0 * v_board.  All entries
// are exact 0/±1 — see header for the -X (180°) axis convention.
static const int8_t kNoseBase[6][9] = {
    // +X: identity
    { 1, 0, 0,   0, 1, 0,   0, 0, 1 },
    // -X: 180° about board +Z
    { -1, 0, 0,   0, -1, 0,   0, 0, 1 },
    // +Y: shortest arc +Y→+X (90° about -Z)
    { 0, 1, 0,   -1, 0, 0,   0, 0, 1 },
    // -Y: shortest arc -Y→+X (90° about +Z)
    { 0, -1, 0,   1, 0, 0,   0, 0, 1 },
    // +Z: shortest arc +Z→+X (90° about +Y)
    { 0, 0, 1,   0, 1, 0,   -1, 0, 0 },
    // -Z: shortest arc -Z→+X (90° about -Y)
    { 0, 0, -1,   0, 1, 0,   1, 0, 0 },
};

// Quarter-turn about rocket +X (CCW, right-hand rule), applied AFTER the
// nose alignment: cos/sin pairs for clock = 0..3.
static const int8_t kClockCos[4] = { 1, 0, -1, 0 };
static const int8_t kClockSin[4] = { 0, 1, 0, -1 };

void orientCodeToMatrix(uint8_t code, float R[9])
{
    if (code >= ORIENT_CODE_COUNT) code = ORIENT_CODE_IDENTITY;
    const int8_t *B = kNoseBase[code / 4];
    const int8_t c = kClockCos[code % 4];
    const int8_t s = kClockSin[code % 4];

    // R = Rx(clock) * B with Rx = [[1,0,0],[0,c,-s],[0,s,c]]:
    // row0 = B row0, row1 = c*B row1 - s*B row2, row2 = s*B row1 + c*B row2.
    for (int j = 0; j < 3; j++)
    {
        R[0 + j] = (float)B[0 + j];
        R[3 + j] = (float)(c * B[3 + j] - s * B[6 + j]);
        R[6 + j] = (float)(s * B[3 + j] + c * B[6 + j]);
    }
}

bool orientMatrixToCode(const float R[9], uint8_t &code)
{
    for (uint8_t cand = 0; cand < ORIENT_CODE_COUNT; cand++)
    {
        float C[9];
        orientCodeToMatrix(cand, C);
        bool match = true;
        for (int i = 0; i < 9; i++)
        {
            if (fabsf(R[i] - C[i]) > 1e-3f) { match = false; break; }
        }
        if (match) { code = cand; return true; }
    }
    return false;
}

void orientMatrixToQuat(const float R[9], float q[4])
{
    // Shepperd's method: pick the largest of the four squared components
    // for numeric stability, then normalize with scalar part >= 0.
    const float r00 = R[0], r01 = R[1], r02 = R[2];
    const float r10 = R[3], r11 = R[4], r12 = R[5];
    const float r20 = R[6], r21 = R[7], r22 = R[8];
    const float tr = r00 + r11 + r22;

    float w, x, y, z;
    if (tr > 0.0f)
    {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (r21 - r12) / s;
        y = (r02 - r20) / s;
        z = (r10 - r01) / s;
    }
    else if (r00 > r11 && r00 > r22)
    {
        float s = sqrtf(1.0f + r00 - r11 - r22) * 2.0f;
        w = (r21 - r12) / s;
        x = 0.25f * s;
        y = (r01 + r10) / s;
        z = (r02 + r20) / s;
    }
    else if (r11 > r22)
    {
        float s = sqrtf(1.0f + r11 - r00 - r22) * 2.0f;
        w = (r02 - r20) / s;
        x = (r01 + r10) / s;
        y = 0.25f * s;
        z = (r12 + r21) / s;
    }
    else
    {
        float s = sqrtf(1.0f + r22 - r00 - r11) * 2.0f;
        w = (r10 - r01) / s;
        x = (r02 + r20) / s;
        y = (r12 + r21) / s;
        z = 0.25f * s;
    }

    float n = sqrtf(w * w + x * x + y * y + z * z);
    if (n < 1e-9f) { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; return; }
    if (w < 0.0f) n = -n;  // canonical sign: scalar part >= 0
    q[0] = w / n; q[1] = x / n; q[2] = y / n; q[3] = z / n;
}

void orientCodeToQuat(uint8_t code, float q[4])
{
    float R[9];
    orientCodeToMatrix(code, R);
    orientMatrixToQuat(R, q);
}

void orientQuatToMatrix(const float q[4], float R[9])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float n = sqrtf(w * w + x * x + y * y + z * z);
    if (n < 1e-9f)
    {
        // Degenerate input: fall back to identity rather than NaN.
        for (int i = 0; i < 9; i++) R[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        return;
    }
    w /= n; x /= n; y /= n; z /= n;

    R[0] = 1.0f - 2.0f * (y * y + z * z);
    R[1] = 2.0f * (x * y - w * z);
    R[2] = 2.0f * (x * z + w * y);
    R[3] = 2.0f * (x * y + w * z);
    R[4] = 1.0f - 2.0f * (x * x + z * z);
    R[5] = 2.0f * (y * z - w * x);
    R[6] = 2.0f * (x * z - w * y);
    R[7] = 2.0f * (y * z + w * x);
    R[8] = 1.0f - 2.0f * (x * x + y * y);
}

uint8_t orientNearestNoseCode(const float v_board[3], float *residual_deg)
{
    const float ax = fabsf(v_board[0]);
    const float ay = fabsf(v_board[1]);
    const float az = fabsf(v_board[2]);
    const float n  = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-9f)
    {
        if (residual_deg) *residual_deg = 0.0f;
        return ORIENT_CODE_IDENTITY;
    }

    int axis = 0;
    float amax = ax;
    if (ay > amax) { axis = 1; amax = ay; }
    if (az > amax) { axis = 2; amax = az; }

    const bool neg = v_board[axis] < 0.0f;
    const uint8_t nose_sel = (uint8_t)(axis * 2 + (neg ? 1 : 0));

    if (residual_deg)
    {
        float c = amax / n;
        if (c > 1.0f) c = 1.0f;
        *residual_deg = acosf(c) * (180.0f / (float)M_PI);
    }
    return (uint8_t)(nose_sel * 4);
}

const char *orientCodeName(uint8_t code)
{
    static const char *names[ORIENT_CODE_COUNT] = {
        "+X",      "+X r90",  "+X r180", "+X r270",
        "-X",      "-X r90",  "-X r180", "-X r270",
        "+Y",      "+Y r90",  "+Y r180", "+Y r270",
        "-Y",      "-Y r90",  "-Y r180", "-Y r270",
        "+Z",      "+Z r90",  "+Z r180", "+Z r270",
        "-Z",      "-Z r90",  "-Z r180", "-Z r270",
    };
    return (code < ORIENT_CODE_COUNT) ? names[code] : "?";
}

void quatFromAccelHeading(float acc_x_frd, float acc_y_frd, float acc_z_frd,
                          float heading_rad, float q[4])
{
    static constexpr float DEG2RAD_f = (float)M_PI / 180.0f;

    float g_mag = sqrtf(acc_x_frd * acc_x_frd +
                        acc_y_frd * acc_y_frd +
                        acc_z_frd * acc_z_frd);
    if (g_mag < 0.1f) g_mag = 9.807f;

    float sx = acc_x_frd / g_mag;
    if (sx > 1.0f) sx = 1.0f;
    if (sx < -1.0f) sx = -1.0f;
    const float pitch_rad = asinf(sx);

    // Roll is ill-conditioned near vertical (Y/Z components vanish);
    // matches the original FC init's 80° gate.
    float roll_rad = 0.0f;
    if (fabsf(pitch_rad) < 80.0f * DEG2RAD_f)
    {
        roll_rad = atan2f(-acc_y_frd, -acc_z_frd);
    }

    // Body-to-NED quaternion from ZYX Euler (yaw, pitch, roll)
    const float cy = cosf(heading_rad * 0.5f), sy = sinf(heading_rad * 0.5f);
    const float cp = cosf(pitch_rad * 0.5f),   sp = sinf(pitch_rad * 0.5f);
    const float cr = cosf(roll_rad * 0.5f),    sr = sinf(roll_rad * 0.5f);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

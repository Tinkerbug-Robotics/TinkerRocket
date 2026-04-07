#ifndef TRIMUINT_H
#define TRIMUINT_H

#include <compat.h>

// IMU data structure
typedef struct 
{
    uint32_t time_micros;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double acc_x;
    double acc_y;
    double acc_z;
    double mag_x;
    double mag_y;
    double mag_z;
} IMU_Data;

class TR_IMU_Int
{
    public:
        TR_IMU_Int();

        // Filter initialization
        void init();

        void update(bool debug_filter_in,
                    bool integrate_pos_vel,
                    bool use_ahrs_acc,
                    IMU_Data imu_data);
                    
        // ### Getter Methods ###
        
        // Get Navigation Estimates
        inline void getPos(float (&response)[3]) 
        {
            response[0] = p_ENU_m_[0];
            response[1] = p_ENU_m_[1];
            response[2] = p_ENU_m_[2];
        }
        inline void getVel(float (&response)[3]) 
        {
            response[0] = v_ENU_ms_[0];
            response[1] = v_ENU_ms_[1];
            response[2] = v_ENU_ms_[2];
        }
        inline void getAccel(float (&response)[3]) 
        {
            response[0] = a_ENU_mps2_[0];
            response[1] = a_ENU_mps2_[1];
            response[2] = a_ENU_mps2_[2];
        }

        inline void getOrientation(float (&response)[3])
        {
            response[0] = euler_body_rad_[0];
            response[1] = euler_body_rad_[1];
            response[2] = euler_body_rad_[2];
        }

        inline void getQuaternion(float (&response)[4])
        {
            response[0] = quat_BL_[0];
            response[1] = quat_BL_[1];
            response[2] = quat_BL_[2];
            response[3] = quat_BL_[3];
        }

    private:

        // ### Class variables ###
        
        // Request to print additional debug data
        bool debug_request_;
        
        // Integrated position estimate (ENU, m)
        float p_ENU_m_[3]; 
        
        // Integrated velocity estimate (ENU, m/s)
        float v_ENU_ms_[3];
        
        // Acceleration estimate in ENU (ENU,m/s^2)
        double a_ENU_mps2_[3];
        
        // Euler angles - roll, pitch & yaw (rad)
        float euler_body_rad_[3];
        
        // Change in time (s)
        float dt_s_;
        
        // Previous time (us)
        uint32_t tPrev_us_;
        
        // Last GNSS time (us)
        uint32_t timeWeekPrev_;
    
        // Quaterion
        float quat_BL_[4];
        
        // Body to ENU transformation matrix
        float T_B2ENU_[3][3];
           
        // ### Global Constants ###

        // Acceleration due to gravity
        const float G = 9.807f;

        const float RAD2DEG = 180.0/PI;
        const float DEG2RAD = PI/180.0;
        
        // Conversions from IMU int data to engineering units
        float dps_conversion;
        float ms2_conversion;
        float uT_conversion;
        
        // ### AHRS Variables ###
        float twoKp;
        float twoKi;  
        float q0;
        float q1;
        float q2;
        float q3;
        float integralFBx;
        float integralFBy;
        float integralFBz;
        bool anglesComputed;
                                                     
        // ### Inline Frequently Called Functions for Runtime Efficiency ###
        
        inline void Quat2DCM_ENU(float DCM[3][3], 
                                 float q[4])
        {
            float q0 = q[0]; // w
            float q1 = q[1]; // x
            float q2 = q[2]; // y
            float q3 = q[3]; // z

            // Convert quaternion to ENU DCM
            DCM[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
            DCM[0][1] = 2.0f * (q1 * q2 + q0 * q3);
            DCM[0][2] = 2.0f * (q1 * q3 - q0 * q2);

            DCM[1][0] = 2.0f * (q1 * q2 - q0 * q3);
            DCM[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
            DCM[1][2] = 2.0f * (q2 * q3 + q0 * q1);

            DCM[2][0] = 2.0f * (q1 * q3 + q0 * q2);
            DCM[2][1] = 2.0f * (q2 * q3 - q0 * q1);
            DCM[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        }
        
        // Mahony AHRS filter
        inline void updateMahony(float quat_BL_[4],
                                 float gx, 
                                 float gy, 
                                 float gz, 
                                 float ax, 
                                 float ay,
                                 float az, 
                                 float mx, 
                                 float my, 
                                 float mz, 
                                 float dt) 
        {
            float recipNorm;
            float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
            float hx, hy, bx, bz;
            float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
            float halfex, halfey, halfez;
            float qa, qb, qc;
            
            // Compute feedback only if accelerometer measurement valid
            // (avoids NaN in accelerometer normalisation)
            if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
            {

                // Use IMU algorithm if magnetometer measurement invalid
                // (avoids NaN in magnetometer normalisation)
                if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
                {

                    // Normalise accelerometer measurement
                    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                    ax *= recipNorm;
                    ay *= recipNorm;
                    az *= recipNorm;

                    // Estimated direction of gravity
                    halfvx = q1 * q3 - q0 * q2;
                    halfvy = q0 * q1 + q2 * q3;
                    halfvz = q0 * q0 - 0.5f + q3 * q3;

                    // Error is sum of cross product between estimated
                    // and measured direction of gravity
                    halfex = (ay * halfvz - az * halfvy);
                    halfey = (az * halfvx - ax * halfvz);
                    halfez = (ax * halfvy - ay * halfvx);

                    // Compute and apply integral feedback if enabled
                    if (twoKi > 0.0f)
                    {
                        // integral error scaled by Ki
                        integralFBx += twoKi * halfex * dt;
                        integralFBy += twoKi * halfey * dt;
                        integralFBz += twoKi * halfez * dt;
                        gx += integralFBx; // apply integral feedback
                        gy += integralFBy;
                        gz += integralFBz;
                    } else {
                        integralFBx = 0.0f; // prevent integral windup
                        integralFBy = 0.0f;
                        integralFBz = 0.0f;
                    }

                    // Apply proportional feedback
                    gx += twoKp * halfex;
                    gy += twoKp * halfey;
                    gz += twoKp * halfez;
                    
                } else {
            
                    // Normalise accelerometer measurement
                    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                    ax *= recipNorm;
                    ay *= recipNorm;
                    az *= recipNorm;

                    // Normalise magnetometer measurement
                    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
                    mx *= recipNorm;
                    my *= recipNorm;
                    mz *= recipNorm;

                    // Auxiliary variables to avoid repeated arithmetic
                    q0q0 = q0 * q0;
                    q0q1 = q0 * q1;
                    q0q2 = q0 * q2;
                    q0q3 = q0 * q3;
                    q1q1 = q1 * q1;
                    q1q2 = q1 * q2;
                    q1q3 = q1 * q3;
                    q2q2 = q2 * q2;
                    q2q3 = q2 * q3;
                    q3q3 = q3 * q3;

                    // Reference direction of Earth's magnetic field
                    hx = 2.0f *
                         (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
                    hy = 2.0f *
                         (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
                    bx = sqrtf(hx * hx + hy * hy);
                    bz = 2.0f *
                         (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

                    // Estimated direction of gravity and magnetic field
                    halfvx = q1q3 - q0q2;
                    halfvy = q0q1 + q2q3;
                    halfvz = q0q0 - 0.5f + q3q3;
                    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
                    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
                    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

                    // Error is sum of cross product between estimated direction
                    // and measured direction of field vectors
                    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
                    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
                    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

                    // Compute and apply integral feedback if enabled
                    if (twoKi > 0.0f) 
                    {
                      // Integral error scaled by Ki
                      integralFBx += twoKi * halfex * dt;
                      integralFBy += twoKi * halfey * dt;
                      integralFBz += twoKi * halfez * dt;
                      gx += integralFBx; // apply integral feedback
                      gy += integralFBy;
                      gz += integralFBz;
                    } else {
                      integralFBx = 0.0f; // prevent integral windup
                      integralFBy = 0.0f;
                      integralFBz = 0.0f;
                    }

                    // Apply proportional feedback
                    gx += twoKp * halfex;
                    gy += twoKp * halfey;
                    gz += twoKp * halfez;
                }
            }

            // Integrate rate of change of quaternion
            gx *= (0.5f * dt); // pre-multiply common factors
            gy *= (0.5f * dt);
            gz *= (0.5f * dt);
            qa = q0;
            qb = q1;
            qc = q2;
            q0 += (-qb * gx - qc * gy - q3 * gz);
            q1 += (qa * gx + qc * gz - q3 * gy);
            q2 += (qa * gy - qb * gz + q3 * gx);
            q3 += (qa * gz + qb * gy - qc * gx);

            // Normalise quaternion
            recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
            q0 *= recipNorm;
            q1 *= recipNorm;
            q2 *= recipNorm;
            q3 *= recipNorm;
            anglesComputed = 0;
            
            quat_BL_[0] = q0;
            quat_BL_[1] = q1;
            quat_BL_[2] = q2;
            quat_BL_[3] = q3;
        }
        
        // Quaternion to Euler angles (ZYX / 3-2-1 decomposition)
        // Body frame: FLU (X=Forward, Y=Left, Z=Up)
        // World frame: ENU (X=East, Y=North, Z=Up)
        // Convention: positive pitch = nose up, positive yaw = nose right
        // On the pad (nose up): roll≈0, pitch≈+90°, yaw≈0
        inline void Quat2Euler(float q[4], float euler[3])
        {

            // Roll (rotation about body X / forward axis)
            // Positive = right wing down
            euler[0] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                             1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));

            // Pitch (rotation about body Y / left axis)
            // Negated so positive = nose up (aerospace convention)
            float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
            if (fabs(sinp) >= 1)
                euler[1] = -copysign(M_PI / 2, sinp);
            else
                euler[1] = -asin(sinp);

            // Yaw (rotation about body Z / up axis)
            // Negated so positive = nose right (aerospace convention)
            euler[2] = -atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                             1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
        }
        
        // Fast inverse square-root
        // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
        inline float invSqrt(float x)
        {
            float halfx = 0.5f * x;
            union 
            {
                float f;
                long i;
            } conv = {x};
            conv.i = 0x5f3759df - (conv.i >> 1);
            conv.f *= 1.5f - (halfx * conv.f * conv.f);
            conv.f *= 1.5f - (halfx * conv.f * conv.f);
            return conv.f;
        }
        
};

#endif
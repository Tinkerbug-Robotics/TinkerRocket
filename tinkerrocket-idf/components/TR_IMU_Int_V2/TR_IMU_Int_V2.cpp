#include "TR_IMU_Int_V2.h"

TR_IMU_Int::TR_IMU_Int()
{}

void TR_IMU_Int::init()   
{
    // Initialize the time (us)
    tPrev_us_ = micros();
    
    // AHRS parameters
    // 2 * proportional gain (Kp)
    twoKp = 2.0f * 0.5f;
    // 2 * integral gain (Ki) — enables slow gyro bias estimation
    twoKi = 2.0f * 0.01f;
    // Initial quaternion: nose-up vertical rocket in FLU body / ENU world.
    // Rotation of -90° about Y puts body X (nose) along world +Z (up).
    // q = [cos(-45°), 0, sin(-45°), 0] = [0.707, 0, -0.707, 0]
    // Gives Euler angles: roll=0°, pitch=+90°, yaw=0° on the pad.
    q0 = 0.707107f;
    q1 = 0.0f;
    q2 = -0.707107f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    anglesComputed = false;

    // Output quaternion: must match Mahony internal state (q0..q3 above)
    quat_BL_[0] = 0.707107f;
    quat_BL_[1] = 0.0f;
    quat_BL_[2] = -0.707107f;
    quat_BL_[3] = 0.0f;
    
    // Initialize posisition, velocity, and acceleration in ENU
    p_ENU_m_[0] = 0.0f;
    p_ENU_m_[1] = 0.0f;
    p_ENU_m_[2] = 0.0f;
    v_ENU_ms_[0] = 0.0f;
    v_ENU_ms_[1] = 0.0f;
    v_ENU_ms_[2] = 0.0f;
    a_ENU_mps2_[0] = 0.0f;
    a_ENU_mps2_[1] = 0.0f;
    a_ENU_mps2_[2] = 0.0f;
    
    // Initialize to false
    debug_request_ = false;
    
}

void TR_IMU_Int::update(bool debug_request_in,
                        bool integrate_pos_vel,
                        bool use_ahrs_acc,
                        IMU_Data imu_data)
{ 
    debug_request_ = debug_request_in;
    
    // Current time (microseconds)
    uint32_t t_us = micros();
    
    // Change in time since last update (sec)
    dt_s_ = ((float)(t_us - tPrev_us_)) / 1e6;
    
    // Save the previous time for next time (usec)
    tPrev_us_ = t_us;
    
    // Catch large dt
    if (dt_s_ > 0.1)
        dt_s_ = 0.1;
    
    // Format IMU data for integration
    
    // Gyro measured rotation rate (Body, rad/s)
    float gyro_body_rps[3];
    gyro_body_rps[0] = imu_data.gyro_x * DEG2RAD;
    gyro_body_rps[1] = imu_data.gyro_y * DEG2RAD;
    gyro_body_rps[2] = imu_data.gyro_z * DEG2RAD;

    // Accelerometer measurements (m/s2)
    float acc_body_mps2[3];
    acc_body_mps2[0] = imu_data.acc_x;
    acc_body_mps2[1] = imu_data.acc_y;
    acc_body_mps2[2] = imu_data.acc_z;

    // Magnetometer measurements (uT)
    float mag_body_uT[3];
    mag_body_uT[0] = imu_data.mag_x;
    mag_body_uT[1] = imu_data.mag_y;
    mag_body_uT[2] = imu_data.mag_z;

    // Magnetometer sanity check: Earth's field is ~25-65 µT depending on
    // location. Accept 15-80 µT range; outside that the hardware is likely
    // reading garbage (saturated, disconnected, or uncalibrated).
    // Passing (0,0,0) makes the Mahony filter fall back to gyro+accel only.
    {
        float mag_mag_sq = mag_body_uT[0] * mag_body_uT[0]
                         + mag_body_uT[1] * mag_body_uT[1]
                         + mag_body_uT[2] * mag_body_uT[2];
        // 15^2 = 225, 80^2 = 6400
        if (mag_mag_sq < 225.0f || mag_mag_sq > 6400.0f)
        {
            mag_body_uT[0] = 0.0f;
            mag_body_uT[1] = 0.0f;
            mag_body_uT[2] = 0.0f;
        }
    }

    // Before flight, AHRS filter uses gyroscope, accelerometer, and
    // magnetometer data to improve orientation estimate.
    // Uses acceleration due to gravity to estimate orientation, which 
    // can't be used during flight since acceleration is not (mostly) 
    // due to gravity during flight.
    if (use_ahrs_acc)
    {
        // Initialize Mahony orientation estimator
        updateMahony(quat_BL_,
                     gyro_body_rps[0],
                     gyro_body_rps[1],
                     gyro_body_rps[2],
                     acc_body_mps2[0],
                     acc_body_mps2[1],
                     acc_body_mps2[2],
                     mag_body_uT[0],
                     mag_body_uT[1],
                     mag_body_uT[2],
                     dt_s_);
    }
    // Set acceleration to zero to integrate gyro data during flight.
    // Ignore acceleration data since the rocket is boosting or falling
    else
    {
        updateMahony(quat_BL_,
                     gyro_body_rps[0],
                     gyro_body_rps[1],
                     gyro_body_rps[2],
                     0.0f,
                     0.0f,
                     0.0f,
                     mag_body_uT[0],
                     mag_body_uT[1],
                     mag_body_uT[2],
                     dt_s_);
    }
    
    // Quaternion updated by Mahony filter — do NOT flip sign.
    // q and -q represent the same rotation, but flipping when q[0]<0
    // causes Euler angle discontinuities near 180° rotations.

    // Euler angles in radians
    Quat2Euler(quat_BL_,euler_body_rad_);
    
    // Compute DCM for Body to ENU frames from Quaternion
    float T_ENU2B[3][3];
    Quat2DCM_ENU(T_ENU2B, quat_BL_);
    
    // Transpose of T_ENU2B saved to class variable
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++) 
        {
            T_B2ENU_[j][i] = T_ENU2B[i][j];
        }
    }
    
    // Transpose acceleration to ENU frame (ENU, m/s^2)
    for (int i = 0; i < 3; i++) 
    {
        a_ENU_mps2_[i] = 0.0f;
        for (int j = 0; j < 3; j++) 
        {
            a_ENU_mps2_[i] += T_B2ENU_[i][j] * acc_body_mps2[j];
        }
    }
    
    // Remove gravity, which acts in the up direction (ENU, m/s^2)
    a_ENU_mps2_[2] -= G;
    
    if (integrate_pos_vel)
    {
    
        // Update velocity given acceleration (ENU, m/s)
        v_ENU_ms_[0] = v_ENU_ms_[0] + a_ENU_mps2_[0] * dt_s_;
        v_ENU_ms_[1] = v_ENU_ms_[1] + a_ENU_mps2_[1] * dt_s_;
        v_ENU_ms_[2] = v_ENU_ms_[2] + a_ENU_mps2_[2] * dt_s_;

        // Update position given velocity (ENU, m/s^2)
        p_ENU_m_[0] = p_ENU_m_[0] + v_ENU_ms_[0] * dt_s_;
        p_ENU_m_[1] = p_ENU_m_[1] + v_ENU_ms_[1] * dt_s_;
        p_ENU_m_[2] = p_ENU_m_[2] + v_ENU_ms_[2] * dt_s_;
    }
    
    // Debug output
    if (debug_request_)
    {
        ESP_LOGD("IMU_INT", "dt_s_: %.5f", dt_s_);
        ESP_LOGD("IMU_INT", "gyro_body_rps: [%.5f, %.5f, %.5f]", gyro_body_rps[0], gyro_body_rps[1], gyro_body_rps[2]);
        ESP_LOGD("IMU_INT", "acc_body_mps2: [%.5f, %.5f, %.5f]", acc_body_mps2[0], acc_body_mps2[1], acc_body_mps2[2]);
        ESP_LOGD("IMU_INT", "mag_body_uT: [%.5f, %.5f, %.5f]", mag_body_uT[0], mag_body_uT[1], mag_body_uT[2]);
        ESP_LOGD("IMU_INT", "quat_BL_: [%.5f, %.5f, %.5f, %.5f]", quat_BL_[0], quat_BL_[1], quat_BL_[2], quat_BL_[3]);
        ESP_LOGD("IMU_INT", "a_ENU_mps2_: [%.5f, %.5f, %.5f]", a_ENU_mps2_[0], a_ENU_mps2_[1], a_ENU_mps2_[2]);
        ESP_LOGD("IMU_INT", "v_ENU_ms_: [%.5f, %.5f, %.5f]", v_ENU_ms_[0], v_ENU_ms_[1], v_ENU_ms_[2]);
        ESP_LOGD("IMU_INT", "p_ENU_m_: [%.5f, %.5f, %.5f]", p_ENU_m_[0], p_ENU_m_[1], p_ENU_m_[2]);
        ESP_LOGD("IMU_INT", "Roll: %.3f  Pitch: %.3f  Yaw: %.3f",
                 euler_body_rad_[0]*RAD2DEG, euler_body_rad_[1]*RAD2DEG, euler_body_rad_[2]*RAD2DEG);
    }

}






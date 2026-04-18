#ifndef TR_PID_H
#define TR_PID_H

#include <compat.h>


class TR_PID
{
public:

    // Constructor
    explicit TR_PID(float kp,
                    float ki,
                    float kd,
                    float max_in,
                    float min_in);

    // Computes dt internally from micros()
    float computePID(float setpoint, float measurment);

    // Platform-independent path: explicit dt in seconds
    float computePID(float setpoint, float measurement, float dt_seconds);

    // Set proportional gain
    void setKp(float Kp);

    // Set integral gain
    void setKi(float Ki);

    // Set derivative gain
    void setKd(float Kd);

    // Configure a 1-pole low-pass filter on the derivative term. Raw
    // backward-difference derivatives amplify measurement noise (sample-to-
    // sample jitter divided by a small dt); this filter rejects that HF
    // noise while preserving phase lead at the frequencies that matter for
    // closed-loop damping. Set fc_hz > 0 to enable, 0 to disable (default).
    // Typical value: 5-20 Hz for PIDs running at ~500 Hz.
    void setDerivativeFilterCutoffHz(float fc_hz);

    // Set output limits
    void setMinCmd(float min_in);
    void setMaxCmd(float max_in);

    // Reset internal state (for replay / test sessions)
    void reset();

    // Reset only the integral accumulator (preserves D-term state)
    void resetIntegral();

protected:

    // Proportional gain
    float Kp;

    // Integral gain
    float Ki;

    // Derivative gain
    float Kd;

    // Accumulate error for integral term
    float cumulative_error;

    uint32_t last_update_time = 0;

    float last_error;
    float last_measurement = 0.0f;

    bool first_call;

    // Min and max allowable controller outputs
    float max_cmd;
    float min_cmd;

    // D-term low-pass filter state. d_filter_fc_hz <= 0 disables filtering
    // (output equals raw backward-difference derivative).
    float d_filter_fc_hz = 0.0f;
    float d_filtered     = 0.0f;

};

#endif // TR_PID_H

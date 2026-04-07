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

};

#endif // TR_PID_H

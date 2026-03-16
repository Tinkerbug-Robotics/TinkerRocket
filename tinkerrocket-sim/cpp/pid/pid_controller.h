#pragma once

#include <algorithm>

/// Platform-independent PID controller.
/// Ported from TR_PID (Arduino), with explicit dt parameter.
class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float min_cmd, float max_cmd)
        : Kp_(kp), Ki_(ki), Kd_(kd),
          min_cmd_(min_cmd), max_cmd_(max_cmd),
          cumulative_error_(0.0f), last_error_(0.0f),
          first_call_(true) {}

    /// Compute PID output given setpoint, measurement, and timestep.
    /// Returns 0 on first call (initialization).
    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;

        if (first_call_ || dt <= 0.0f) {
            first_call_ = false;
            last_error_ = error;
            return 0.0f;
        }

        // Proportional
        float P = Kp_ * error;

        // Integral with anti-windup (clamp integral term)
        cumulative_error_ += error * dt;
        float I = std::clamp(Ki_ * cumulative_error_, min_cmd_, max_cmd_);

        // Derivative
        float D = Kd_ * ((error - last_error_) / dt);

        last_error_ = error;

        return std::clamp(P + I + D, min_cmd_, max_cmd_);
    }

    /// Reset controller state.
    void reset() {
        cumulative_error_ = 0.0f;
        last_error_ = 0.0f;
        first_call_ = true;
    }

    // Gain setters
    void setKp(float kp) { Kp_ = kp; }
    void setKi(float ki) { Ki_ = ki; }
    void setKd(float kd) { Kd_ = kd; }
    void setMinCmd(float v) { min_cmd_ = v; }
    void setMaxCmd(float v) { max_cmd_ = v; }

    // Gain getters
    float getKp() const { return Kp_; }
    float getKi() const { return Ki_; }
    float getKd() const { return Kd_; }
    float getMinCmd() const { return min_cmd_; }
    float getMaxCmd() const { return max_cmd_; }
    float getCumulativeError() const { return cumulative_error_; }
    float getLastError() const { return last_error_; }

private:
    float Kp_, Ki_, Kd_;
    float min_cmd_, max_cmd_;
    float cumulative_error_;
    float last_error_;
    bool first_call_;
};

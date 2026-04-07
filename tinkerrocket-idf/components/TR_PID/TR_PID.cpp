#include "TR_PID.h"

TR_PID::TR_PID(float kp,
               float ki,
               float kd,
               float max_in,
               float min_in)
{
    // Set gains based on inputs
    Kp = kp;
    Ki = ki;
    Kd = kd;

    // Set min and max controller output
    max_cmd = max_in;
    min_cmd = min_in;

    // Cumulative error starts at 0
    cumulative_error = 0.0;

    // Initialize the last error to 0
    last_error = 0.0;

    // Indicate it is the first time calling the library
    first_call = true;
}

float TR_PID::computePID(float setpoint, float actual)
{
    // Current time in micro seconds
    uint32_t now = micros();

    // Delta time since last call
    float dt = (last_update_time > 0)
             ? (now - last_update_time) / 1e6f : 0.0f;

    // Save timestamp
    last_update_time = now;

    // Delegate to the dt-based implementation
    return computePID(setpoint, actual, dt);
}

float TR_PID::computePID(float setpoint, float actual, float dt_seconds)
{
    float dt = dt_seconds;

    // Difference between desired state and input state is the error
    float error = setpoint - actual;

    // Initialize and return if it is the first call
    // Return if dt is negative
    if (first_call || dt <= 0.0f)
    {
        first_call = false;

        // Save data for next time
        last_error = error;
        last_measurement = actual;
        return 0;
    }

    // Proportional component of output
    float P = Kp * error;

    // Integral component of output
    cumulative_error += error * dt;
    float I = constrain(Ki * cumulative_error, min_cmd, max_cmd);

    // Derivative-on-measurement to avoid kick on setpoint change.
    // Uses negative sign because d(measurement)/dt opposes d(error)/dt.
    float D = -Kd * ((actual - last_measurement) / dt);

    // Calculate command out
    float command_out = P + I + D;

    // Save data for next time
    last_error = error;
    last_measurement = actual;

    return constrain(command_out, min_cmd, max_cmd);
}

void TR_PID::setKp(float kp)
{
    Kp = kp;
}

void TR_PID::setKi(float ki)
{
    Ki = ki;
}

void TR_PID::setKd(float kd)
{
    Kd = kd;
}

void TR_PID::setMinCmd(float min_in)
{
    min_cmd = min_in;
}

void TR_PID::setMaxCmd(float max_in)
{
    max_cmd = max_in;
}

void TR_PID::reset()
{
    cumulative_error = 0.0;
    last_error = 0.0;
    last_measurement = 0.0;
    last_update_time = 0;
    first_call = true;
}

void TR_PID::resetIntegral()
{
    cumulative_error = 0.0;
}

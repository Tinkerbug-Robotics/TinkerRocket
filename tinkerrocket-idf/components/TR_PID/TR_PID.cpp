#include "TR_PID.h"

#include <cmath>

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

    // I term starts at 0
    integral_term = 0.0;

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
        d_filtered = 0.0f;
        return 0;
    }

    // Proportional component of output
    float P = Kp * error;

    // Integral component of output. Conditional-integration anti-windup:
    // skip accumulation while |error| exceeds the separation threshold so a
    // large transient (e.g. a launch roll kick) can't wind up the integrator.
    // The accumulator holds its value through the transient and resumes near
    // the setpoint to reject steady disturbances. threshold <= 0 => always
    // integrate (original behavior).
    //
    // The accumulator is the I term itself: it sums Ki * error * dt, where
    // this used to sum error * dt and multiply by the current Ki.  With a
    // fixed Ki the two are the same controller.  They part when Ki moves,
    // which the roll V² gain schedule does on every tick: Ki * sum(e*dt)
    // rescales everything already integrated, so as a coasting rocket slows
    // and Ki grows (up to 3x), a fin-trim offset held by the integrator grows
    // with it and has to be unwound all the way to apogee — a standing roll
    // error of about -trim * (dKi/dt) / Ki^2.  A trim built into the airframe
    // scales with V² exactly as the fins' authority does, so what cancels it
    // is one fin angle at every speed; summing Ki * e * dt holds that angle
    // and lets Ki change only how fast the I term moves from here on.
    float abs_error = (error < 0.0f) ? -error : error;
    if (integral_sep_threshold <= 0.0f || abs_error <= integral_sep_threshold)
    {
        // Ki is inside the state now, so a non-finite gain or error would
        // stick until the next reset (Ki * sum(e*dt) recovered on the next
        // finite tick).  Skip the increment instead.
        const float increment = Ki * error * dt;
        if (std::isfinite(increment))
        {
            integral_term += increment;
        }
    }
    // #386: clamp the ACCUMULATOR, not just the I output.  With only the
    // output clamped, a long saturated stretch grows the accumulator far past
    // the value that already pins I at max_cmd; after the error reverses, all
    // that surplus must be integrated back down before I (and the command)
    // moves at all — fins held hard-over long past reversal.  Held in output
    // units, the output-saturating value is simply [min_cmd, max_cmd], so
    // recovery begins on the first post-reversal sample and no Ki == 0 guard
    // is needed.
    integral_term = constrain(integral_term, min_cmd, max_cmd);
    float I = integral_term;

    // Derivative-on-measurement to avoid kick on setpoint change.
    // Uses negative sign because d(measurement)/dt opposes d(error)/dt.
    float D_raw = -Kd * ((actual - last_measurement) / dt);

    // Optional 1-pole LP filter on the derivative term to reject high-
    // frequency measurement noise that would otherwise saturate the PID
    // output and cause servo flutter. alpha = dt / (dt + tau), where
    // tau = 1/(2*pi*fc). fc_hz <= 0 disables the filter.
    float D;
    if (d_filter_fc_hz > 0.0f)
    {
        const float tau   = 1.0f / (2.0f * 3.14159265358979323846f * d_filter_fc_hz);
        const float alpha = dt / (dt + tau);
        d_filtered += alpha * (D_raw - d_filtered);
        D = d_filtered;
    }
    else
    {
        D = D_raw;
    }

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
    // Ki = 0 is the operator saying "no integral action", and under
    // Ki * sum(e*dt) it took the I term to zero at once.  Keep that: holding the
    // I term in output units would otherwise freeze it at its last value.
    if (ki == 0.0f)
    {
        integral_term = 0.0f;
    }
}

void TR_PID::setKd(float kd)
{
    Kd = kd;
}

void TR_PID::setDerivativeFilterCutoffHz(float fc_hz)
{
    d_filter_fc_hz = (fc_hz > 0.0f) ? fc_hz : 0.0f;
    d_filtered = 0.0f;
}

void TR_PID::setIntegralSeparationThreshold(float threshold)
{
    integral_sep_threshold = (threshold > 0.0f) ? threshold : 0.0f;
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
    integral_term = 0.0;
    last_error = 0.0;
    last_measurement = 0.0;
    last_update_time = 0;
    first_call = true;
    d_filtered = 0.0f;
}

void TR_PID::resetIntegral()
{
    integral_term = 0.0;
}

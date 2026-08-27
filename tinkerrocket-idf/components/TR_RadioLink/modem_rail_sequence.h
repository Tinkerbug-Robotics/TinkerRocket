#pragma once

#include <cstddef>

// Ordering rules for gating the LoRa daughterboard's rail (#700).
//
// Every daughterboard branch on V8 and V9 gates the RAIL, never the signals,
// and there is no series resistance on any of these lines. So a host TX pad
// that is still configured while the rail is down idles HIGH at 3.3 V and
// feeds the module through its RX ESD diode:
//
//   V8  low-side N-FET in the ground return -> the module's ground FLOATS
//   V9  high-side TPS22810              -> ground is common, the rail is DEAD
//
// Different mechanism, identical outcome: the host powers a module it believes
// it switched off, unbounded, for as long as the rail stays down. On the FC's
// RunCam that was most of the flight before it was fixed; the LoRa path had
// the same hole in every failClosed() and re-attach failure.
//
// The rules are trivial and that is the point — they were violated because
// nothing wrote them down, not because they are hard. Pinned here so a new
// failure path that drops the rail cannot quietly skip the park.
namespace modem_rail {

enum class Step : unsigned char {
    ParkUart,    // pins -> input, no pulls: stop driving
    RailDown,    // act_pin low
    RailUp,      // act_pin high
    AttachUart,  // uart matrix -> pins: start driving
};

// Signals go quiet BEFORE the rail goes away.
inline constexpr Step kPowerDown[] = {Step::ParkUart, Step::RailDown};
// Rail comes up BEFORE the signals do.
inline constexpr Step kPowerUp[]   = {Step::RailUp,   Step::AttachUart};

inline constexpr size_t kPowerDownLen = sizeof(kPowerDown) / sizeof(kPowerDown[0]);
inline constexpr size_t kPowerUpLen   = sizeof(kPowerUp)   / sizeof(kPowerUp[0]);

// The invariant the two sequences exist to hold: the host must never be
// driving the UART while the rail is down. Modelled so a test can assert it
// over any sequence rather than trusting the two arrays by inspection.
struct RailModel {
    bool rail_up      = false;
    bool uart_driving = false;
    bool violated     = false;   // latched: driving observed with rail down

    constexpr void step(Step s)
    {
        switch (s) {
        case Step::ParkUart:   uart_driving = false; break;
        case Step::RailDown:   rail_up      = false; break;
        case Step::RailUp:     rail_up      = true;  break;
        case Step::AttachUart: uart_driving = true;  break;
        }
        if (uart_driving && !rail_up) violated = true;
    }
};

}  // namespace modem_rail

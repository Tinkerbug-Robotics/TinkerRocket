/*
 * Launch and burnout from the IMU, as the P4 detects them to switch the receiver's loops to the boost
 * profile and back (rx_set_boost). On samples one a millisecond of the specific force along the thrust
 * axis:
 *   launch   above launch_ms2 for launch_ms consecutive samples; any sample at or below the bar starts
 *            the count again;
 *   burnout  below zero (the motor spent, drag pulling back) for burnout_ms consecutive samples, not
 *            looked for in the first lockout_ms after launch. The boost profile then stays on for hold_s
 *            more, and narrows from there;
 *   rest     within rest_ms2 of 1 g for rest_ms consecutive samples after the lockout: a false start (a
 *            knock on the pad), and the profile goes off at once. A rocket in flight never reads a steady
 *            1 g along its axis: it reads the thrust while it burns and drag, backwards, after.
 *
 * The default is the receiver's own fast trigger (owner, 2026-10-02): 20 m/s^2 for 20 ms, so the loops
 * widen about 20 ms into the burn. A false trigger here costs only a moment of wider loops. The flight
 * computer's IMU-only rules (tinkerrocket-idf TR_KinematicChecks and BurnoutDetector.h: 30 m/s^2 for
 * 250 ms, no rest exit) are slower on purpose, since its false launch must never arm anything; at 250 ms
 * the unaided loops lose carrier at ignition. boost_detect_fc() gives them, for comparison.
 *
 * The flight computer's launch test reads the force's magnitude. This one reads the axial component:
 * after burnout, drag reads up to several g along the axis, backwards, and must not count as a new burn.
 * From the pad the two are the same. A second burn after the hold (a staged motor) is detected as the
 * first was.
 */
#ifndef GNSS_BOOST_DETECT_H
#define GNSS_BOOST_DETECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float launch_ms2;      /* launch: the axial specific force above this, m/s^2 ... */
    uint16_t launch_ms;    /* ... for this many consecutive samples */
    uint16_t lockout_ms;   /* no burnout in the first this many samples after launch */
    uint16_t burnout_ms;   /* burnout: the axial specific force below zero for this many samples */
    float hold_s;          /* the boost profile stays on this long past burnout */
    float rest_ms2;        /* a false start: within this of 1 g ... */
    uint16_t rest_ms;      /* ... for this many samples after the lockout (0: no rest exit) */
} boost_detect_cfg_t;

typedef enum {
    BOOST_PAD = 0,         /* before launch */
    BOOST_BURN,            /* launch detected; looking for burnout */
    BOOST_HOLD,            /* burnout detected; the profile holds hold_s */
    BOOST_COAST            /* after the hold; a new burn is detected as launch was */
} boost_phase_t;

typedef struct {
    boost_detect_cfg_t cfg;
    boost_phase_t phase;
    uint32_t count;        /* consecutive samples passing the phase's test */
    uint32_t rest;         /* consecutive samples at rest, in the burn */
    uint32_t ms;           /* samples since the phase began */
} boost_detect_t;

/* The receiver's fast trigger: 20 m/s^2 for 20 samples, a 200 ms lockout, below zero for 50, a 2 s
 * hold, and back to rest within 5 m/s^2 of 1 g for 500. */
void boost_detect_default(boost_detect_cfg_t *c);

/* The flight computer's IMU-only rules: 30 m/s^2 for 250 samples, the same burnout and hold, no rest exit. */
void boost_detect_fc(boost_detect_cfg_t *c);

void boost_detect_init(boost_detect_t *d, const boost_detect_cfg_t *c);

/* One IMU sample, a millisecond after the last: the specific force along the thrust axis, m/s^2 (about
 * +9.8 at rest on the pad). Returns 1 while the boost profile should be on, from launch to hold_s past
 * burnout. */
int boost_detect_step(boost_detect_t *d, float f_axial);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_BOOST_DETECT_H */

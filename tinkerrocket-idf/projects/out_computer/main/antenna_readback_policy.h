#pragma once

// The GNSS antenna offset (BLE cmd 81) in the OC's config readback: which
// copy it serves, the FC's own from its config report ("antsrc" "fc") or the
// OC's cache of what the phone last sent ("oc").
//
// The apps take a readback into the rocket's profile only when it is the
// FC's, and a later Send all pushes the profile back to the rocket. So "fc"
// must never carry a value the FC is about to be told to replace: the apps
// would write it over the owner's setting and send it back. Two moments do
// exactly that, and the cache is served through both:
//
//   - The FC has no record of its own (F_ANTENNA_FROM_NVS clear: a fresh
//     board, or its NVS erased) and reports its default, Unknown, while the
//     cache is measured. That is when the OC's self-heal sends it the cache,
//     so the cache stands until the FC remembers a setting: the rule the
//     orientation's "set" follows (sendImuOrientation). With the cache
//     Unknown too, both say the same and the FC's is served.
//   - An antenna command the OC staged is not yet confirmed. The FC sends its
//     first report before it takes a command queued with its rail off, so
//     that report still holds the old value. The command is unconfirmed until
//     a report carries the value staged, or until the command has left the
//     OC's queue for good (served out, or dropped) and a report after that
//     still does not: an FC that refused it, or never got it, then shows its
//     own (reportSettles).
//
// Without a report that carries the antenna (none since the OC booted, or an
// FC older than v4), or with the FC's rail off, there is nothing of the FC's
// to serve.
//
// Pure so the table is host-testable; main.cpp (antennaReadback, the config
// report's handler) gathers the inputs and owns the locking.

namespace AntennaReadbackPolicy {

struct Inputs
{
    bool rail_on            = false;  // the FC's rail is on (pwr_pin_on)
    bool report_valid       = false;  // a config report has arrived since the OC booted
    bool report_has_antenna = false;  // and it is v4 or later, so it carries the antenna
    bool fc_remembers       = false;  // its F_ANTENNA_FROM_NVS: the FC holds a record of its own
    bool staged_unconfirmed = false;  // an antenna command the OC staged is not yet confirmed
    bool cache_known        = false;  // the OC's cache is a measured offset
};

// True: serve the FC's report ("fc"). False: serve the OC's cache ("oc").
inline bool serveFc(const Inputs& in)
{
    if (!in.rail_on || !in.report_valid || !in.report_has_antenna) return false;   // nothing of the FC's
    if (in.staged_unconfirmed) return false;   // the FC has yet to take what the cache holds
    return in.fc_remembers || !in.cache_known;
}

// Does a report that carries the antenna settle an unconfirmed command? When
// it holds the value staged, or when the command has left the queue for good
// (none queued, none being served) and the FC still reports something else.
inline bool reportSettles(bool report_holds_staged, bool command_left_queue)
{
    return report_holds_staged || command_left_queue;
}

}  // namespace AntennaReadbackPolicy

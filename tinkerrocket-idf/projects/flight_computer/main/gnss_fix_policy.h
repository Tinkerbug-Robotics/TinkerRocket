// gnss_fix_policy.h -- which receiver record is a fix the apogee and landing
// votes may count (#1763).  Pure logic, no ESP-IDF, host-tested
// (tests_cpp/test_gnss_fix_policy.cpp).
//
// Three of the four GNSS drivers publish a record on every epoch whether or not
// the receiver has a fix: the SAM-M10Q's NAV-PVT, the PX1105R's 0xDF and the
// mosaic-G5's PVTGeodetic keep coming at their full rate, and the loss shows
// only as fix_mode 0 (a velocity of 0 for the PX1105R and the mosaic-G5).  The
// PX100 on the 65mm Roll Control flight (BARC 2026-11-21) lost its fix from
// +1.65 to +8.95 s and sent 20 records a second all through it, every one
// fix_mode 0 with a vertical velocity of 0.
//
// main.cpp used to hand every such record to the kinematic checks as new GPS.
// Each one stamped the GPS voter fresh, and its velocity never read a descent,
// so a receiver with no fix stayed an available GPS voter that never passed.
// Above the barometer's range the only other voter is the accelerometer: two
// available and one passing, so the quorum of two could not form, and the
// one-sensor-left call (#1706) needs exactly one available.  Apogee waited for
// the barometer to read back in range on the way down (about 23 s late,
// #1711) or for the fix to come back.
//
// Now a record counts for the vote only with a usable fix: the test the
// GNSS ascent gate admits GNSS on (main.cpp, GnssAscentGate's gnss_fix_ok),
// so the record that admitted GNSS and the records that vote meet the same
// bar.  The voter's freshness then measures the time since the last fix: a
// receiver that loses its fix drops out of the vote 500 ms later, as a silent
// one does, and the accelerometer left alone calls apogee.
//
// The GNSS board's driver hands over only fixes it screens as usable (3-D,
// 6 or more satellites; SdrRawFeed.h sdrFixUsable), so on it nothing changes.
#pragma once

#include <cstdint>

namespace gnss_fix
{

// A usable fix: a 3-D fix with at least min_sats satellites
// (config::GNSS_MIN_SATS).  Every driver marks a 3-D fix 3 (the u-blox's GNSS
// plus dead reckoning is 4); fix_mode 2 (a 2-D fix) and 1 (the PX1105R's
// prediction, the u-blox's dead reckoning alone) are not usable.  The u-blox's
// 5 (time only) passes, as in every other fix test in main.cpp; a SAM-M10Q in
// its flight configuration does not report one.
//
// main.cpp sets the kinematic checks' new_gps from this on every record, so it
// describes the LATEST record (the one whose velocity the checks read): a
// no-fix record that lands before the checks have read a usable one clears it.
inline bool usable(uint8_t fix_mode, uint8_t num_sats, uint8_t min_sats)
{
    return fix_mode >= 3u && num_sats >= min_sats;
}

}  // namespace gnss_fix

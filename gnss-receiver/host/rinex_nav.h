/*
 * RINEX 3 navigation files (mixed or single-system): the broadcast ephemerides
 * of GPS, Galileo (I/NAV) and BeiDou, and the header's GPS Klobuchar
 * parameters, as the flight computer could preload them into the receiver.
 * Host only.
 */
#ifndef GNSS_HOST_RINEX_NAV_H
#define GNSS_HOST_RINEX_NAV_H

#include "gnss/eph.h"
#include "gnss/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * For every satellite of the systems in sys_mask (bit 1 << gnss_sys_t), loads the record
 * whose toe is nearest GPS week `week`, second `tow` into eph[sys][prn]. BeiDou GEO
 * satellites, whose orbits need their own transformation, are left out. iono (may be NULL)
 * receives the header's GPSA/GPSB. Returns the number of satellites loaded, or -1 if the
 * file cannot be read.
 */
int rinex_nav_load(const char *path, unsigned sys_mask, int week, double tow, gps_eph_t eph[][GNSS_MAX_PRN + 1],
                   gps_iono_t *iono);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_RINEX_NAV_H */

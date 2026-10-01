#include "rinex_nav.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GPS_TO_BDT_WEEKS 1356  /* BDT week 0 began on GPS week 1356 */

/* A D19.12 field (Fortran D exponents allowed); blank reads as 0. */
static double field(const char *line, int col)
{
    char buf[20];
    size_t len = strlen(line);
    if ((size_t)col >= len) {
        return 0.0;
    }
    size_t n = len - (size_t)col < 19 ? len - (size_t)col : 19;
    memcpy(buf, line + col, n);
    buf[n] = 0;
    for (size_t k = 0; k < n; k++) {
        if (buf[k] == 'D' || buf[k] == 'd') {
            buf[k] = 'E';
        }
    }
    return atof(buf);
}

/* Days from 1980-01-06 (a Sunday) to the date. */
static long days_from_1980(int y, int m, int d)
{
    /* Days from civil (Howard Hinnant's algorithm), relative to 1970-01-01, then shifted. */
    y -= m <= 2;
    long era = (y >= 0 ? y : y - 399) / 400;
    long yoe = y - era * 400;
    long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468 - 3657;
}

int rinex_nav_load(const char *path, unsigned sys_mask, int week, double tow, gps_eph_t eph[][GNSS_MAX_PRN + 1],
                   gps_iono_t *iono)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    char line[256], rec[8][256];
    double best[GNSS_SYS_COUNT][GNSS_MAX_PRN + 1];
    for (int s = 0; s < GNSS_SYS_COUNT; s++) {
        for (int p = 0; p <= GNSS_MAX_PRN; p++) {
            best[s][p] = 1e30;
        }
    }
    if (iono) {
        memset(iono, 0, sizeof(*iono));
    }
    int have_a = 0, have_b = 0;
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "END OF HEADER")) {
            break;
        }
        if (iono && strstr(line, "IONOSPHERIC CORR") && (!strncmp(line, "GPSA", 4) || !strncmp(line, "GPSB", 4))) {
            double *v = line[3] == 'A' ? iono->alpha : iono->beta;
            for (int k = 0; k < 4; k++) {
                char buf[13];
                memcpy(buf, line + 5 + 12 * k, 12);
                buf[12] = 0;
                for (int j = 0; j < 12; j++) {
                    if (buf[j] == 'D') {
                        buf[j] = 'E';
                    }
                }
                v[k] = atof(buf);
            }
            have_a |= line[3] == 'A';
            have_b |= line[3] == 'B';
        }
    }
    if (iono) {
        iono->valid = have_a && have_b;
    }
    const double t_target_gps = (double)week * 604800.0 + tow;
    int loaded = 0;
    while (fgets(rec[0], sizeof(rec[0]), f)) {
        char sysc = rec[0][0];
        int ncont = (sysc == 'R' || sysc == 'S') ? 3 : 7;
        int ok = 1;
        for (int k = 1; k <= ncont; k++) {
            if (!fgets(rec[k], sizeof(rec[k]), f)) {
                ok = 0;
            }
        }
        if (!ok) {
            break;
        }
        int sys = sysc == 'G' ? GNSS_SYS_GPS : (sysc == 'E' ? GNSS_SYS_GAL : (sysc == 'C' ? GNSS_SYS_BDS : -1));
        if (sys < 0 || !(sys_mask & (1u << sys))) {
            continue;
        }
        int prn = atoi((char[3]){rec[0][1], rec[0][2], 0});
        int y, mo, d, hh, mi, ss;
        if (prn < 1 || prn > GNSS_MAX_PRN || sscanf(rec[0] + 4, "%d %d %d %d %d %d", &y, &mo, &d, &hh, &mi, &ss) != 6) {
            continue;
        }
        if (sys == GNSS_SYS_BDS && (prn <= 5 || prn >= 59)) {
            continue;  /* GEO */
        }
        double src = field(rec[5], 23);
        if (sys == GNSS_SYS_GAL && !(((long)src & 1) || ((long)src & 4))) {
            continue;  /* F/NAV: its clock is for E5a */
        }
        gps_eph_t e;
        memset(&e, 0, sizeof(e));
        e.sys = sys;
        e.prn = prn;
        long days = days_from_1980(y, mo, d);
        e.toc = (double)(days % 7) * 86400.0 + hh * 3600.0 + mi * 60.0 + ss;
        e.af0 = field(rec[0], 23);
        e.af1 = field(rec[0], 42);
        e.af2 = field(rec[0], 61);
        e.iode = (int)field(rec[1], 4);
        e.crs = field(rec[1], 23);
        e.delta_n = field(rec[1], 42);
        e.m0 = field(rec[1], 61);
        e.cuc = field(rec[2], 4);
        e.e = field(rec[2], 23);
        e.cus = field(rec[2], 42);
        e.sqrt_a = field(rec[2], 61);
        e.toe = field(rec[3], 4);
        e.cic = field(rec[3], 23);
        e.omega0 = field(rec[3], 42);
        e.cis = field(rec[3], 61);
        e.i0 = field(rec[4], 4);
        e.crc = field(rec[4], 23);
        e.omega = field(rec[4], 42);
        e.omega_dot = field(rec[4], 61);
        e.idot = field(rec[5], 4);
        e.week = (int)field(rec[5], 42);
        e.health = (int)field(rec[6], 23);
        if (sys == GNSS_SYS_GPS) {
            e.tgd = field(rec[6], 42);
            e.iodc = (int)field(rec[6], 61);
        } else if (sys == GNSS_SYS_GAL) {
            e.tgd = field(rec[6], 61);  /* BGD(E1,E5b): the I/NAV clock's single-frequency E1 term */
        } else {
            e.tgd = field(rec[6], 42);  /* TGD1 (B1): SignalSim delays B1C by it */
        }
        double t_rec = (double)e.week * 604800.0 + e.toe;
        double t_want = sys == GNSS_SYS_BDS
                            ? (double)(week - GPS_TO_BDT_WEEKS) * 604800.0 + tow + BDT_MINUS_GPST
                            : t_target_gps;
        double dist = fabs(t_rec - t_want);
        if (dist < best[sys][prn]) {
            best[sys][prn] = dist;
            if (!eph[sys][prn].valid) {
                loaded++;
            }
            e.valid = 1;
            eph[sys][prn] = e;
        }
    }
    fclose(f);
    return loaded;
}

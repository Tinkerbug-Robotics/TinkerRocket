/*
 * The loop-profile argument the host tools share (trksim, gnssrx):
 * PF,PP,PD/LF,LP,LD[:MS] = pull-in FLL, PLL and DLL bandwidths (Hz) / the locked ones, and the
 * FLL's block length in ms (1, 2, 4, 5 or 10; default 1).
 */
#ifndef GNSS_HOST_LOOPS_ARG_H
#define GNSS_HOST_LOOPS_ARG_H

#include <stdio.h>

#include "gnss/trk.h"

static inline int loops_arg_parse(const char *s, trk_profile_t *p)
{
    int ms = 1;
    int n = sscanf(s, "%f,%f,%f/%f,%f,%f:%d", &p->pullin.fll_bw, &p->pullin.pll_bw, &p->pullin.dll_bw,
                   &p->locked.fll_bw, &p->locked.pll_bw, &p->locked.dll_bw, &ms);
    if (n < 6 || (ms != 1 && ms != 2 && ms != 4 && ms != 5 && ms != 10)) {
        return -1;
    }
    p->fll_ms = (unsigned char)ms;
    return 0;
}

static inline void loops_arg_format(const trk_profile_t *p, char *buf, size_t n)
{
    snprintf(buf, n, "%g,%g,%g/%g,%g,%g:%d", (double)p->pullin.fll_bw, (double)p->pullin.pll_bw,
             (double)p->pullin.dll_bw, (double)p->locked.fll_bw, (double)p->locked.pll_bw, (double)p->locked.dll_bw,
             p->fll_ms);
}

#endif /* GNSS_HOST_LOOPS_ARG_H */

#include "gnss/boost_detect.h"

#define BD_G 9.80665f

void boost_detect_default(boost_detect_cfg_t *c)
{
    c->launch_ms2 = 20.0f;
    c->launch_ms = 20;
    c->lockout_ms = 200;
    c->burnout_ms = 50;
    c->hold_s = 2.0f;
    c->rest_ms2 = 5.0f;
    c->rest_ms = 500;
}

void boost_detect_fc(boost_detect_cfg_t *c)
{
    boost_detect_default(c);
    c->launch_ms2 = 30.0f;
    c->launch_ms = 250;
    c->rest_ms = 0;
}

void boost_detect_init(boost_detect_t *d, const boost_detect_cfg_t *c)
{
    d->cfg = *c;
    d->phase = BOOST_PAD;
    d->count = 0;
    d->rest = 0;
    d->ms = 0;
}

static void enter(boost_detect_t *d, boost_phase_t p)
{
    d->phase = p;
    d->count = 0;
    d->rest = 0;
    d->ms = 0;
}

int boost_detect_step(boost_detect_t *d, float f_axial)
{
    d->ms++;
    switch (d->phase) {
    case BOOST_PAD:
    case BOOST_COAST:
        d->count = f_axial > d->cfg.launch_ms2 ? d->count + 1 : 0;
        if (d->count >= d->cfg.launch_ms) {
            enter(d, BOOST_BURN);
        }
        break;
    case BOOST_BURN:
        if (d->ms > d->cfg.lockout_ms) {
            d->count = f_axial < 0.0f ? d->count + 1 : 0;
            if (d->count >= d->cfg.burnout_ms) {
                enter(d, BOOST_HOLD);
                break;
            }
            /* A false start: back at rest. */
            const float off = f_axial - BD_G;
            d->rest = d->cfg.rest_ms > 0 && off < d->cfg.rest_ms2 && off > -d->cfg.rest_ms2 ? d->rest + 1 : 0;
            if (d->cfg.rest_ms > 0 && d->rest >= d->cfg.rest_ms) {
                enter(d, BOOST_PAD);
            }
        }
        break;
    case BOOST_HOLD:
        if (d->ms >= (uint32_t)(d->cfg.hold_s * 1000.0f + 0.5f)) {
            enter(d, BOOST_COAST);
        }
        break;
    }
    return d->phase == BOOST_BURN || d->phase == BOOST_HOLD;
}

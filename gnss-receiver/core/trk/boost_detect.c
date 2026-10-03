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
    c->gate = 0;
    c->ign_s = 1.0f;
    c->tail_frac = 0.9f;
    c->tail_ms = 10;
}

void boost_detect_fc(boost_detect_cfg_t *c)
{
    boost_detect_default(c);
    c->launch_ms2 = 30.0f;
    c->launch_ms = 250;
    c->rest_ms = 0;
}

void boost_detect_gate(boost_detect_cfg_t *c, float ign_s, float tail_frac, uint16_t tail_ms)
{
    c->gate = 1;
    c->ign_s = ign_s;
    c->tail_frac = tail_frac;
    c->tail_ms = tail_ms;
}

void boost_detect_init(boost_detect_t *d, const boost_detect_cfg_t *c)
{
    d->cfg = *c;
    d->phase = BOOST_PAD;
    d->count = 0;
    d->rest = 0;
    d->ms = 0;
    d->peak = 0.0f;
    d->tail = 0;
    d->tailing = 0;
}

static void enter(boost_detect_t *d, boost_phase_t p)
{
    d->phase = p;
    d->count = 0;
    d->rest = 0;
    d->ms = 0;
    if (p == BOOST_BURN) {
        d->peak = 0.0f;
        d->tail = 0;
        d->tailing = 0;
    }
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
        /* The thrust's tail-off: under tail_frac of the burn's peak, uninterrupted. */
        if (f_axial > d->peak) {
            d->peak = f_axial;
        }
        d->tail = f_axial < d->cfg.tail_frac * d->peak ? d->tail + 1 : 0;
        if (d->tail >= d->cfg.tail_ms) {
            d->tailing = 1;
        }
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
    if (d->cfg.gate && d->phase == BOOST_BURN) {
        return d->ms <= (uint32_t)(d->cfg.ign_s * 1000.0f + 0.5f) || d->tailing;
    }
    return d->phase == BOOST_BURN || d->phase == BOOST_HOLD;
}

//swing detection state machine, see swing_detect.h

#include "swing_detect.h"

uint32_t isqrt32(uint32_t x)
{
    uint32_t res = 0, bit = 1uL << 30;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) { x -= res + bit; res = (res >> 1) + bit; }
        else                { res >>= 1; }
        bit >>= 2;
    }
    return res;
}

void swing_init(swing_detector_t *d, const swing_cfg_t *cfg)
{
    d->cfg = *cfg;
    d->state = SW_IDLE;
    d->start2 = (uint32_t)cfg->start_raw * cfg->start_raw;
    d->end2 = (uint32_t)cfg->end_raw * cfg->end_raw;
    d->run = 0;
    d->len = 0;
    d->count = 0;
}

static uint32_t mag2(const int16_t g[3])
{
    //max is 3 * 32768^2 = 3.2e9 which still fits in a uint32
    uint32_t s = 0;
    for (int i = 0; i < 3; i++) {
        int32_t v = g[i];
        s += (uint32_t)(v * v);
    }
    return s;
}

bool swing_update(swing_detector_t *d, uint32_t seq, const int16_t gyr[3],
                  uint8_t sample_flags, swing_event_t *ev)
{
    uint32_t m2 = mag2(gyr);

    switch (d->state) {
    case SW_IDLE:
        d->run = (m2 > d->start2) ? (uint16_t)(d->run + 1) : 0;
        if (d->run >= d->cfg.start_samples) {
            d->state = SW_IN_SWING;
            d->start_seq = seq - (d->cfg.start_samples - 1u);
            d->len = d->cfg.start_samples;
            d->peak_mag2 = m2;
            d->peak_seq = seq;
            d->flags = sample_flags & FLAG_GYRO_CLIPPED;
            d->run = 0;
        }
        return false;

    case SW_IN_SWING:
        d->len++;
        d->flags |= sample_flags & FLAG_GYRO_CLIPPED;
        if (m2 > d->peak_mag2) { d->peak_mag2 = m2; d->peak_seq = seq; }
        d->run = (m2 < d->end2) ? (uint16_t)(d->run + 1) : 0;

        if (d->len > d->cfg.max_samples) {           //too long to be a swing
            d->state = SW_COOLDOWN;
            d->run = 0;
            return false;
        }
        if (d->run >= d->cfg.end_samples) {           //swing is over
            uint32_t active_len = d->len - d->run;    //don't count the quiet part at the end
            d->state = SW_COOLDOWN;
            d->run = 0;
            if (active_len < d->cfg.min_samples) return false;  //too short
            d->count++;
            uint32_t peak = isqrt32(d->peak_mag2);
            ev->swing_id = d->count;
            ev->start_seq = d->start_seq;
            ev->peak_seq = d->peak_seq;
            ev->end_seq = seq - d->cfg.end_samples;  //last sample above END
            ev->peak_mag_raw = (uint16_t)(peak > 0xFFFFu ? 0xFFFFu : peak);
            ev->flags = d->flags;
            return true;
        }
        return false;

    case SW_COOLDOWN:
    default:
        if (++d->run >= d->cfg.cooldown_samples) {
            d->state = SW_IDLE;
            d->run = 0;
        }
        return false;
    }
}

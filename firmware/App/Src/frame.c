//packs data into uart packets, format is in frame.h

#include "frame.h"

uint8_t frame_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t *put_u8(uint8_t *p, uint8_t v)   { *p++ = v; return p; }
static uint8_t *put_u16(uint8_t *p, uint16_t v) { *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8); return p; }
static uint8_t *put_u32(uint8_t *p, uint32_t v)
{
    *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8); *p++ = (uint8_t)(v >> 16); *p++ = (uint8_t)(v >> 24);
    return p;
}

//writes sync + type + len, returns where the payload starts
static uint8_t *begin(uint8_t *out, uint8_t type, uint8_t len)
{
    out[0] = FRAME_SYNC0;
    out[1] = FRAME_SYNC1;
    out[2] = type;
    out[3] = len;
    return &out[4];
}

//adds the crc over type + len + payload, returns the total length
static size_t finish(uint8_t *out, uint8_t len)
{
    out[4 + len] = frame_crc8(&out[2], (size_t)len + 2u);
    return (size_t)len + FRAME_OVERHEAD;
}

size_t frame_encode_sample(uint8_t *out, const imu_sample_t *s)
{
    uint8_t *p = begin(out, FRAME_TYPE_SAMPLE, FRAME_SAMPLE_LEN);
    p = put_u32(p, s->seq);
    p = put_u32(p, s->t_cyc);
    for (int i = 0; i < 3; i++) p = put_u16(p, (uint16_t)s->acc[i]);
    for (int i = 0; i < 3; i++) p = put_u16(p, (uint16_t)s->gyr[i]);
    (void)put_u8(p, s->flags);
    return finish(out, FRAME_SAMPLE_LEN);
}

size_t frame_encode_status(uint8_t *out, const status_info_t *st)
{
    uint8_t *p = begin(out, FRAME_TYPE_STATUS, FRAME_STATUS_LEN);
    p = put_u32(p, st->core_hz);
    p = put_u32(p, st->uptime_ms);
    p = put_u32(p, st->ticks);
    p = put_u32(p, st->missed_reads);
    p = put_u32(p, st->ring_overflows);
    p = put_u32(p, st->tx_dropped);
    p = put_u32(p, st->spi_errors);
    p = put_u32(p, st->swing_count);
    p = put_u32(p, st->rate_mhz);
    p = put_u16(p, st->gyro_fs_dps);
    p = put_u8(p, st->acc_fs_g);
    p = put_u8(p, st->bmi_status);
    p = put_u8(p, st->chip_id);
    p = put_u8(p, st->bmi_diag);
    p = put_u8(p, 0); (void)put_u8(p, 0);
    return finish(out, FRAME_STATUS_LEN);
}

size_t frame_encode_swing(uint8_t *out, const swing_event_t *sw)
{
    uint8_t *p = begin(out, FRAME_TYPE_SWING, FRAME_SWING_LEN);
    p = put_u32(p, sw->swing_id);
    p = put_u32(p, sw->start_seq);
    p = put_u32(p, sw->peak_seq);
    p = put_u32(p, sw->end_seq);
    p = put_u16(p, sw->peak_mag_raw);
    (void)put_u8(p, sw->flags);
    return finish(out, FRAME_SWING_LEN);
}

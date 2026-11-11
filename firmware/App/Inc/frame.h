//uart packet format sent to the pc
//every packet: 0xAA 0x55 | type | len | payload | crc8
//crc8 (poly 0x07, init 0) covers type, len and payload, numbers are little endian
//binary instead of text because a text sample is about 2x bigger
//python/swingproto.py has to match this file

#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#include <stddef.h>

#define FRAME_SYNC0 0xAAu
#define FRAME_SYNC1 0x55u

#define FRAME_TYPE_SAMPLE 0x01u //one imu reading
#define FRAME_TYPE_STATUS 0x02u //health counters, once a second
#define FRAME_TYPE_SWING  0x03u //one detected swing

#define FRAME_OVERHEAD    5u    //2 sync + type + len + crc
#define FRAME_SAMPLE_LEN  21u
#define FRAME_STATUS_LEN  44u
#define FRAME_SWING_LEN   19u
#define FRAME_MAX_BYTES   (FRAME_OVERHEAD + FRAME_STATUS_LEN)

//flag bits for samples and swings
#define FLAG_GYRO_CLIPPED 0x01u //a gyro axis hit the limit, real value may be higher
#define FLAG_ACC_CLIPPED  0x02u

typedef struct {
    uint32_t seq;     //timer tick number, gaps on the pc = dropped samples
    uint32_t t_cyc;   //cycle counter when the timer fired, for jitter
    int16_t  acc[3];  //raw accel counts xyz
    int16_t  gyr[3];  //raw gyro counts xyz
    uint8_t  flags;
} imu_sample_t;

typedef struct {
    uint32_t core_hz;         //cpu clock, to turn t_cyc into time
    uint32_t uptime_ms;
    uint32_t ticks;           //timer ticks so far
    uint32_t missed_reads;    //timer fired while the last spi read was still going
    uint32_t ring_overflows;  //main loop too slow, sample thrown away
    uint32_t tx_dropped;      //uart too slow, packet thrown away
    uint32_t spi_errors;
    uint32_t swing_count;
    uint32_t rate_mhz;        //configured timer rate in millihertz (setting, not measured)
    uint16_t gyro_fs_dps;     //gyro full scale
    uint8_t  acc_fs_g;        //accel full scale
    uint8_t  bmi_status;      //0 = ok, see bmi270.h
    uint8_t  chip_id;         //should be 0x24
    uint8_t  bmi_diag;
    uint8_t  reserved[2];
} status_info_t;

typedef struct {
    uint32_t swing_id;
    uint32_t start_seq;
    uint32_t peak_seq;
    uint32_t end_seq;
    uint16_t peak_mag_raw;    //peak |gyro| in raw counts, deg/s = raw * fs / 32768
    uint8_t  flags;
} swing_event_t;

uint8_t frame_crc8(const uint8_t *data, size_t len);

//each one writes a full packet into out[] and returns its length
size_t frame_encode_sample(uint8_t *out, const imu_sample_t *s);
size_t frame_encode_status(uint8_t *out, const status_info_t *st);
size_t frame_encode_swing(uint8_t *out, const swing_event_t *sw);

#endif

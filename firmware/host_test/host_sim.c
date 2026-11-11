//runs the firmware's packet encoder and swing detector on a pc
//reads "seq gx gy gz" lines (raw gyro counts) and writes the same packets the board would send
//python/test_host.py uses it to check the c and python code agree. tests logic only, no hardware

#include <stdio.h>
#include <stdint.h>
#include "frame.h"
#include "swing_detect.h"
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

//has to match SWING_CFG in App/Src/app.c and SwingCfg in python/swingproto.py
#define DPS_TO_RAW(dps) ((uint16_t)((dps) * 32768u / 2000u))
static const swing_cfg_t CFG = {
    .start_raw = DPS_TO_RAW(300), .end_raw = DPS_TO_RAW(150),
    .start_samples = 5, .end_samples = 50, .min_samples = 50,
    .max_samples = 2000, .cooldown_samples = 300, .skip_axis = 1,
};

int main(void)
{
#ifdef _WIN32
    //windows stdout is text mode and would turn every 0x0A byte into 0x0D 0x0A
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    swing_detector_t d;
    swing_init(&d, &CFG);
    uint8_t f[FRAME_MAX_BYTES];
    unsigned long seq;
    int gx, gy, gz;
    while (scanf("%lu %d %d %d", &seq, &gx, &gy, &gz) == 4) {
        imu_sample_t s = {0};
        s.seq = (uint32_t)seq;
        s.t_cyc = (uint32_t)(seq * 180000u);
        s.gyr[0] = (int16_t)gx; s.gyr[1] = (int16_t)gy; s.gyr[2] = (int16_t)gz;
        for (int i = 0; i < 3; i++)
            if (s.gyr[i] == INT16_MAX || s.gyr[i] == INT16_MIN) s.flags |= FLAG_GYRO_CLIPPED;
        fwrite(f, 1, frame_encode_sample(f, &s), stdout);
        swing_event_t ev;
        if (swing_update(&d, s.seq, s.gyr, s.flags, &ev))
            fwrite(f, 1, frame_encode_swing(f, &ev), stdout);
    }
    status_info_t st = {0};
    st.core_hz = 180000000u; st.swing_count = d.count; st.gyro_fs_dps = 2000; st.acc_fs_g = 16;
    st.chip_id = 0x24; st.rate_mhz = 1000000u;
    fwrite(f, 1, frame_encode_status(f, &st), stdout);
    return 0;
}

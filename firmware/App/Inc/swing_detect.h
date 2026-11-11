//swing detection state machine
//IDLE: waiting, a swing starts when the gyro stays above START for a few samples
//IN_SWING: keeps the fastest spin (the peak), ends when the gyro stays below END
//COOLDOWN: ignores the follow through so it isn't counted as a second swing
//START is higher than END so a signal sitting at one threshold doesn't flicker
//plain c with no hal so the same code runs on the pc tests

#ifndef SWING_DETECT_H
#define SWING_DETECT_H

#include <stdint.h>
#include <stdbool.h>
#include "frame.h"

typedef struct {
    uint16_t start_raw;     //|gyro| above this can start a swing (raw counts)
    uint16_t end_raw;       //|gyro| below this can end a swing
    uint16_t start_samples; //samples in a row above start
    uint16_t end_samples;   //samples in a row below end
    uint16_t min_samples;   //shorter ones are ignored (bumps, taps)
    uint16_t max_samples;   //longer ones are ignored (twirling the racket)
    uint16_t cooldown_samples;
    uint8_t  skip_axis;     //gyro axis left out of |gyro| (0 x, 1 y, 2 z, 3 none)
} swing_cfg_t;

typedef enum { SW_IDLE = 0, SW_IN_SWING, SW_COOLDOWN } swing_state_t;

typedef struct {
    swing_cfg_t   cfg;
    swing_state_t state;
    uint32_t start2, end2;  //thresholds squared so there's no sqrt every sample
    uint16_t run;           //samples in a row above/below the threshold
    uint32_t len;           //samples since the swing started
    uint32_t start_seq;
    uint32_t peak_mag2;
    uint32_t peak_seq;
    uint8_t  flags;
    uint32_t count;         //swings reported so far
} swing_detector_t;

void swing_init(swing_detector_t *d, const swing_cfg_t *cfg);

//feed one sample, returns true and fills *ev when a swing just ended
bool swing_update(swing_detector_t *d, uint32_t seq, const int16_t gyr[3],
                  uint8_t sample_flags, swing_event_t *ev);

//integer square root so the pc and stm32 get the exact same answer
uint32_t isqrt32(uint32_t x);

#endif

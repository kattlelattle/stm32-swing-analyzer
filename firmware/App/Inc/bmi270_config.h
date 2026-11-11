//bosch's bmi270 config file (see bmi270_config.c)

#ifndef BMI270_CONFIG_H
#define BMI270_CONFIG_H

#include <stdint.h>

extern const uint8_t bmi270_config_file[];
extern const uint16_t bmi270_config_size;

//returns 0 if the config array is missing
int bmi270_config_is_real(void);

#endif

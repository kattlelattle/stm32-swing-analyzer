//bmi270 driver over spi (stm32 hal)
//register values are from the bmi270 datasheet, init reads them back to catch mistakes

#ifndef BMI270_H
#define BMI270_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

//registers
#define BMI270_REG_CHIP_ID         0x00u
#define BMI270_REG_ERR             0x02u
#define BMI270_REG_STATUS          0x03u
#define BMI270_REG_DATA_START      0x0Cu  //acc x lsb, 0x0C..0x17 = acc xyz then gyro xyz
#define BMI270_REG_INTERNAL_STATUS 0x21u
#define BMI270_REG_ACC_CONF        0x40u
#define BMI270_REG_ACC_RANGE       0x41u
#define BMI270_REG_GYR_CONF        0x42u
#define BMI270_REG_GYR_RANGE       0x43u
#define BMI270_REG_INIT_CTRL       0x59u
#define BMI270_REG_INIT_DATA       0x5Eu
#define BMI270_REG_PWR_CONF        0x7Cu
#define BMI270_REG_PWR_CTRL        0x7Du
#define BMI270_REG_CMD             0x7Eu

#define BMI270_CHIP_ID_VALUE       0x24u
#define BMI270_SPI_READ            0x80u  //bit 7 set = read

//settings
//the bmi270 has no 1000 hz rate, so it runs at 1600 hz and the stm32 reads the newest value 1000x/s
#define BMI270_ACC_CONF_VALUE  0xACu  //perf filter, normal bandwidth, 1600 hz
#define BMI270_GYR_CONF_VALUE  0xECu  //perf filter + noise perf, normal bandwidth, 1600 hz
#define BMI270_ACC_RANGE_VALUE 0x03u  //+-16 g, largest range since swings are violent
#define BMI270_GYR_RANGE_VALUE 0x00u  //+-2000 deg/s, largest range
#define BMI270_ACC_FS_G        16u
#define BMI270_GYR_FS_DPS      2000u

//one burst read is [address][dummy][12 data bytes]
//in spi mode the bmi270 sends one junk byte before the real data
#define BMI270_BURST_DATA_LEN  12u
#define BMI270_BURST_XFER_LEN  (2u + BMI270_BURST_DATA_LEN)

typedef enum {
    BMI270_OK = 0,
    BMI270_ERR_SPI = 1,         //hal spi call failed
    BMI270_ERR_CHIP_ID = 2,     //wrong chip id: wiring, power or cs
    BMI270_ERR_INIT = 3,        //config file upload not accepted
    BMI270_ERR_READBACK = 4,    //a setting didn't stick
    BMI270_ERR_NO_CONFIG = 5,   //bmi270_config.c has no config array
    BMI270_ERR_ERRREG = 6       //sensor reported a fatal error
} bmi270_status_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint8_t chip_id;            //last chip id read, for debugging
    uint8_t diag;               //register value that was wrong (or err_reg after init)
} bmi270_t;

bmi270_status_t bmi270_init(bmi270_t *dev);
bmi270_status_t bmi270_read_regs(bmi270_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
bmi270_status_t bmi270_write_reg(bmi270_t *dev, uint8_t reg, uint8_t value);

//fills tx[] for the dma burst read, done once at startup
void bmi270_prepare_burst(uint8_t tx[BMI270_BURST_XFER_LEN]);

//turns the received burst into accel and gyro counts
void bmi270_parse_burst(const uint8_t rx[BMI270_BURST_XFER_LEN], int16_t acc[3], int16_t gyr[3]);

#endif

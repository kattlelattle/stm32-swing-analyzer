//bmi270 driver over spi
//init follows the datasheet quick start:
//1. dummy read to switch the chip from i2c to spi, then read the chip id
//2. turn off advanced power save
//3. upload bosch's 8 kb config file to INIT_DATA
//4. check INTERNAL_STATUS says init ok
//5. turn on accel + gyro and set rate and range

#include "bmi270.h"
#include "bmi270_config.h"
#include <string.h>

#define SPI_TIMEOUT_MS 10u

static inline void cs_low(bmi270_t *d)  { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(bmi270_t *d) { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET); }

bmi270_status_t bmi270_read_regs(bmi270_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr = (uint8_t)(reg | BMI270_SPI_READ);
    HAL_StatusTypeDef r;

    cs_low(dev);
    r = HAL_SPI_Transmit(dev->hspi, &addr, 1, SPI_TIMEOUT_MS);
    if (r == HAL_OK) {
        uint8_t dummy;
        r = HAL_SPI_Receive(dev->hspi, &dummy, 1, SPI_TIMEOUT_MS);  //junk byte
    }
    if (r == HAL_OK) {
        r = HAL_SPI_Receive(dev->hspi, buf, len, SPI_TIMEOUT_MS);
    }
    cs_high(dev);
    return (r == HAL_OK) ? BMI270_OK : BMI270_ERR_SPI;
}

bmi270_status_t bmi270_write_reg(bmi270_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };
    cs_low(dev);
    HAL_StatusTypeDef r = HAL_SPI_Transmit(dev->hspi, tx, 2, SPI_TIMEOUT_MS);
    cs_high(dev);
    //before the config is loaded the chip needs ~450 us between writes
    HAL_Delay(1);
    return (r == HAL_OK) ? BMI270_OK : BMI270_ERR_SPI;
}

static bmi270_status_t write_burst(bmi270_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t addr = (uint8_t)(reg & 0x7Fu);
    cs_low(dev);
    HAL_StatusTypeDef r = HAL_SPI_Transmit(dev->hspi, &addr, 1, SPI_TIMEOUT_MS);
    if (r == HAL_OK) {
        //hal wants a non-const pointer but doesn't change the data
        r = HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, len, 1000u);
    }
    cs_high(dev);
    return (r == HAL_OK) ? BMI270_OK : BMI270_ERR_SPI;
}

//write a register and read it back to make sure it stuck
static bmi270_status_t write_check(bmi270_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t rb = 0;
    if (bmi270_write_reg(dev, reg, value) != BMI270_OK) return BMI270_ERR_SPI;
    if (bmi270_read_regs(dev, reg, &rb, 1) != BMI270_OK) return BMI270_ERR_SPI;
    if (rb != value) { dev->diag = rb; return BMI270_ERR_READBACK; }
    return BMI270_OK;
}

bmi270_status_t bmi270_init(bmi270_t *dev)
{
    uint8_t v = 0;
    bmi270_status_t s;

    dev->diag = 0;
    cs_high(dev);
    HAL_Delay(2);                        //sensor power up time

    //1. the chip wakes up in i2c mode and a cs edge switches it to spi, so the first read is junk
    (void)bmi270_read_regs(dev, BMI270_REG_CHIP_ID, &v, 1);
    HAL_Delay(1);
    s = bmi270_read_regs(dev, BMI270_REG_CHIP_ID, &v, 1);
    dev->chip_id = v;
    if (s != BMI270_OK) return s;
    if (v != BMI270_CHIP_ID_VALUE) return BMI270_ERR_CHIP_ID;

    //chip id is checked before the config so wiring can be tested on its own
    if (!bmi270_config_is_real()) return BMI270_ERR_NO_CONFIG;

    //2. advanced power save off so writes can go fast
    if (bmi270_write_reg(dev, BMI270_REG_PWR_CONF, 0x00) != BMI270_OK) return BMI270_ERR_SPI;

    //3. upload the config file
    if (bmi270_write_reg(dev, BMI270_REG_INIT_CTRL, 0x00) != BMI270_OK) return BMI270_ERR_SPI;
    if (write_burst(dev, BMI270_REG_INIT_DATA, bmi270_config_file, bmi270_config_size) != BMI270_OK)
        return BMI270_ERR_SPI;
    if (bmi270_write_reg(dev, BMI270_REG_INIT_CTRL, 0x01) != BMI270_OK) return BMI270_ERR_SPI;

    //4. datasheet says wait at least 20 ms, then INTERNAL_STATUS[3:0] should be 1
    HAL_Delay(25);
    if (bmi270_read_regs(dev, BMI270_REG_INTERNAL_STATUS, &v, 1) != BMI270_OK) return BMI270_ERR_SPI;
    if ((v & 0x0Fu) != 0x01u) { dev->diag = v; return BMI270_ERR_INIT; }

    //5. accel + gyro + temp on, rate and range, power save stays off
    if ((s = write_check(dev, BMI270_REG_PWR_CTRL, 0x0E)) != BMI270_OK) return s;
    if ((s = write_check(dev, BMI270_REG_ACC_CONF, BMI270_ACC_CONF_VALUE)) != BMI270_OK) return s;
    if ((s = write_check(dev, BMI270_REG_GYR_CONF, BMI270_GYR_CONF_VALUE)) != BMI270_OK) return s;
    if ((s = write_check(dev, BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_VALUE)) != BMI270_OK) return s;
    if ((s = write_check(dev, BMI270_REG_GYR_RANGE, BMI270_GYR_RANGE_VALUE)) != BMI270_OK) return s;
    if ((s = write_check(dev, BMI270_REG_PWR_CONF, 0x02)) != BMI270_OK) return s;

    HAL_Delay(5);   //let the first samples arrive

    //ERR_REG: bit 0 fatal_err, bits 1-4 internal_err, bit 6 fifo_err, bit 7 aux_err (bosch bmi2_defs.h)
    //only fatal_err means the chip can't be used, the full value goes out in diag
    //reading it right after setup without the delay gave 0x04, after the delay it's 0
    if (bmi270_read_regs(dev, BMI270_REG_ERR, &v, 1) != BMI270_OK) return BMI270_ERR_SPI;
    dev->diag = v;
    if (v & 0x01u) return BMI270_ERR_ERRREG;

    return BMI270_OK;
}

void bmi270_prepare_burst(uint8_t tx[BMI270_BURST_XFER_LEN])
{
    memset(tx, 0, BMI270_BURST_XFER_LEN);
    tx[0] = BMI270_REG_DATA_START | BMI270_SPI_READ;
}

void bmi270_parse_burst(const uint8_t rx[BMI270_BURST_XFER_LEN], int16_t acc[3], int16_t gyr[3])
{
    const uint8_t *d = &rx[2];  //skip the address byte and the dummy byte
    for (int i = 0; i < 3; i++) {
        acc[i] = (int16_t)((uint16_t)d[2 * i] | ((uint16_t)d[2 * i + 1] << 8));
        gyr[i] = (int16_t)((uint16_t)d[6 + 2 * i] | ((uint16_t)d[6 + 2 * i + 1] << 8));
    }
}

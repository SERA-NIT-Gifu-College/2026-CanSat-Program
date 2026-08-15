#include "imu.h"

#include <hardware/i2c.h>
#include <pico/time.h>
#include <stdbool.h>

#define WRITE_REG(reg, data) \
    { \
        uint8_t buff[2] = {(reg), (data)}; \
        i2c_write_blocking(imu->inst, imu->addr, buff, 2, false); \
    } 
#define READ_REG(reg, dest) \
    { \
        uint8_t buff = (reg); \
        i2c_write_blocking(imu->inst, imu->addr, &buff, 1, true); \
        i2c_read_blocking(imu->inst, imu->addr, (dest), 1, false); \
    }

void setupIMU(BNO055 *imu) {
    BNO055_Axis_CFG axis;
    axis.cfg = P1;
    WRITE_REG(PAGE_ID, 0x00)
    sleep_ms(100);
    WRITE_REG(OPR_MODE, CONFIG_MODE)
    sleep_ms(100);
    WRITE_REG(UNIT_SEL, 0x80)
    WRITE_REG(AXIS_MAP_CONFIG, axis.config[0])
    WRITE_REG(AXIS_MAP_SIGN, axis.config[1])
    WRITE_REG(OPR_MODE, imu->primary_mode)
    sleep_ms(100);
}

void saveCalibration(BNO055 *imu, uint8_t *dest) {
    WRITE_REG(OPR_MODE, CONFIG_MODE)
    sleep_ms(100);

    uint8_t reg = CALIB_DATA_START;
    i2c_write_blocking(imu->inst, imu->addr, &reg, 1, true);
    i2c_read_blocking(imu->inst, imu->addr, dest, 22, false);

    WRITE_REG(OPR_MODE, imu->primary_mode)
    sleep_ms(100);
}

void loadCalibration(BNO055 *imu, uint8_t *src) {
    WRITE_REG(OPR_MODE, CONFIG_MODE)
    sleep_ms(100);

    uint8_t reg = CALIB_DATA_START;
    i2c_write_blocking(imu->inst, imu->addr, &reg, 1, true);
    i2c_write_blocking(imu->inst, imu->addr, src, 22, false);

    WRITE_REG(OPR_MODE, imu->primary_mode)
    sleep_ms(100);
}

void readIMU(BNO055 *imu) {
    uint8_t reg = EUL_DATA_START;
    i2c_write_blocking(imu->inst, imu->addr, &reg, 1, true);
    i2c_read_blocking(imu->inst, imu->addr, (uint8_t*)imu->euler, 6, false);
}

void obtainStatus(BNO055 *imu, uint8_t *calib, uint8_t *ssc, uint8_t *ser) {
    READ_REG(CALIB_STAT, calib)
    READ_REG(SYS_STATUS, ssc)
    READ_REG(SYS_ERR, ser)
}

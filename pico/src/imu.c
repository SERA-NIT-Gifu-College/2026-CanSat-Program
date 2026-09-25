#include "imu.h"

#include <hardware/i2c.h>
#include <hardware/flash.h>
#include <pico/flash.h>
#include <pico/time.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define DEBUG_IMU

#define WRITE_REG(reg, data) \
    { \
        uint8_t buff[2] = {(reg), (data)}; \
        int ret = i2c_write_blocking(imu->inst, imu->addr, buff, 2, false); \
        if (ret == PICO_ERROR_GENERIC) return 1; \
    } 
#define READ_REG(reg, dest) \
    { \
        uint8_t buff = (reg); \
        int ret = i2c_write_blocking(imu->inst, imu->addr, &buff, 1, true); \
        if (ret == PICO_ERROR_GENERIC) return 1; \
        ret = i2c_read_blocking(imu->inst, imu->addr, (dest), 1, false); \
        if (ret == PICO_ERROR_GENERIC) return 1; \
    }

#define FLASH_TARGET_OFFSET (384 * 1024)

static const uint8_t *flash_target = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);

static void flash_write_cb(void *param) {
    uint32_t ofst = ((uintptr_t*)param)[0];
    const uint8_t *data = (const uint8_t*)(((uintptr_t*)param)[1]);
    flash_range_program(ofst, data, FLASH_PAGE_SIZE);
}
static void flash_erase_cb(void *param) {
    uint32_t ofst = (uint32_t)param;
    flash_range_erase(ofst, FLASH_SECTOR_SIZE);
}

int setupIMU(BNO055 *imu) {
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

    return 0;
}

int saveCalibration(BNO055 *imu) {
    WRITE_REG(OPR_MODE, CONFIG_MODE)
    sleep_ms(100);

    uint8_t buff[22] = {0};

    uint8_t reg = CALIB_DATA_START;
    int ret = i2c_write_blocking(imu->inst, imu->addr, &reg, 1, true);
    if (ret == PICO_ERROR_GENERIC) return 1;
    ret = i2c_read_blocking(imu->inst, imu->addr, buff, 22, false);
    if (ret == PICO_ERROR_GENERIC) return 1;

    int rc = flash_safe_execute(flash_erase_cb, (void*)FLASH_TARGET_OFFSET, 400);
    hard_assert(rc == PICO_OK);

    uintptr_t p[] = {FLASH_TARGET_OFFSET, (uintptr_t)buff};
    rc = flash_safe_execute(flash_write_cb, p, 400);
    hard_assert(rc == PICO_OK);

#ifdef DEBUG_IMU
    for (int i = 0; i < 22; i++) {
        printf("0x%02X ", flash_target[i]);
    }
    printf("\r\n");
#endif

    WRITE_REG(OPR_MODE, imu->primary_mode)
    sleep_ms(100);

    return 0;
}

int loadCalibration(BNO055 *imu) {
    WRITE_REG(OPR_MODE, CONFIG_MODE)
    sleep_ms(100);

    uint8_t buff[22] = {0};
    memcpy(&(buff[0]), flash_target, 22);

#ifdef DEBUG_IMU
    for (int i = 0; i < 22; i++) {
        printf("0x%02X ", flash_target[i]);
    }
    printf("\r\n");
#endif

    uint8_t reg = CALIB_DATA_START;
    int ret = i2c_write_blocking(imu->inst, imu->addr, &reg, 1, true);
    if (ret == PICO_ERROR_GENERIC) return 1;
    ret = i2c_write_blocking(imu->inst, imu->addr, buff, 22, false);
    if (ret == PICO_ERROR_GENERIC) return 1;

    WRITE_REG(OPR_MODE, imu->primary_mode)
    sleep_ms(100);

    return 0;
}

int readIMU(BNO055 *imu, uint16_t euler[3]) {
    uint8_t reg = EUL_DATA_START;
    int ret = i2c_write_blocking(imu->inst, imu->addr, &reg, 1, true);
    if (ret == PICO_ERROR_GENERIC) return 1;
    ret = i2c_read_blocking(imu->inst, imu->addr, (uint8_t*)(&(euler[0])), 6, false);
    if (ret == PICO_ERROR_GENERIC) return 1;

    return 0;
}

int obtainStatus(BNO055 *imu, uint8_t *calib, uint8_t *ssc, uint8_t *ser) {
    READ_REG(CALIB_STAT, calib)
    READ_REG(SYS_STATUS, ssc)
    READ_REG(SYS_ERR, ser)

    return 0;
}

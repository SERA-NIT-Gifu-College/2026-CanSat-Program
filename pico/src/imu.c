#include "imu.h"

#include <hardware/i2c.h>
#include <hardware/flash.h>
#include <pico/flash.h>
#include <pico/time.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define DEBUG_IMU 0

#define WRITE_REG(reg, data) \
    { \
        if (DEBUG_IMU) { \
            printf("IMU W: 0x%02X, 0x%02X\r\n", (reg), (data)); \
        } \
        uint8_t buff[2] = {(reg), (data)}; \
        int ret = i2c_write_blocking(imu->inst, imu->addr, buff, 2, false); \
        if (ret == PICO_ERROR_GENERIC) return 1; \
    } 
#define READ_REG(reg, dest) \
    { \
        if (DEBUG_IMU) { \
            printf("IMU R: 0x%02X\r\n", (reg)); \
        } \
        uint8_t buff = (reg); \
        int ret = i2c_write_blocking(imu->inst, imu->addr, &buff, 1, true); \
        if (ret == PICO_ERROR_GENERIC) return 1; \
        ret = i2c_read_blocking(imu->inst, imu->addr, (dest), 1, false); \
        if (ret == PICO_ERROR_GENERIC) return 1; \
    }

#define FLASH_CALIB_TARGET_OFFSET (448 * 1024)
#define FLASH_LAUNCH_TELEM_TARGET_OFFSET (512 * 1024)

const uint8_t *flash_calib_target = (const uint8_t*)(XIP_BASE + FLASH_CALIB_TARGET_OFFSET);
const uint8_t *flash_launch_telem_target = (const uint8_t*)(XIP_BASE + FLASH_LAUNCH_TELEM_TARGET_OFFSET);

static void flash_write_cb(void *param) {
    uint32_t ofst = ((uintptr_t*)param)[0];
    uint32_t sz = ((uintptr_t*)param)[1];
    const uint8_t *data = (const uint8_t*)(((uintptr_t*)param)[2]);
    flash_range_program(ofst, data, sz);
}
static void flash_erase_cb(void *param) {
    uint32_t ofst = ((uintptr_t*)param)[0];
    uint32_t sz = ((uintptr_t*)param)[1];
    flash_range_erase(ofst, sz);
}

int setupIMULaunchPhase(BNO055 *imu, int mode) {
    BNO055_Axis_CFG axis;
    axis.cfg = P1;
    WRITE_REG(PAGE_ID, 0x00)
    sleep_ms(100);
    WRITE_REG(OPR_MODE, CONFIG_MODE)
    sleep_ms(100);
    WRITE_REG(UNIT_SEL, 0x80)
    WRITE_REG(AXIS_MAP_CONFIG, axis.config[0])
    WRITE_REG(AXIS_MAP_SIGN, axis.config[1])
    WRITE_REG(PAGE_ID, 0x01)
    sleep_ms(100);
    if (mode == 0) {
        // ACC_HIGH_G
        WRITE_REG(INT_MSK, 0x20)
        WRITE_REG(INT_EN, 0x20)
        // 10 ms
        WRITE_REG(ACC_HG_DURATION, 0x04)
        // Default, > 12G
        WRITE_REG(ACC_HG_THRES, 0xC0)
        // Enable all axis for HG, Disable rest
        WRITE_REG(ACC_INT_SETTINGS, 0xE0)
    } else if (mode == 1) {
        // ACC_NM
        WRITE_REG(INT_MSK, 0x80)
        WRITE_REG(INT_EN, 0x80)
        // Enable all axis for NM, Disable rest
        WRITE_REG(ACC_INT_SETTINGS, 0x1C)
        // < 0.03125 G
        WRITE_REG(ACC_NM_THRES, 0x01)
        // NM mode, 16 seconds timeout
        WRITE_REG(ACC_NM_SET, (0x0F << 1) | 0x05)
    }
    WRITE_REG(PAGE_ID, 0x00)
    sleep_ms(100);
    WRITE_REG(OPR_MODE, IMU)
    sleep_ms(100);

    return 0;
}

int setupIMU(BNO055 *imu) {
    BNO055_Axis_CFG axis;
    axis.config[0] = 0x12;
    axis.config[1] = 0x04;
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

int clearINT(BNO055 *imu) {
    WRITE_REG(PAGE_ID, 0x00)
    sleep_ms(100);
    WRITE_REG(SYS_TRIGGER, 0x40)
    sleep_ms(100);

    return 0;
}

int getAcc(BNO055 *imu, uint8_t *buff) {
    uint8_t reg = ACC_DATA_START;
    int ret = i2c_write_blocking(imu->inst, imu->addr, &reg, 1, true);
    if (ret == PICO_ERROR_GENERIC) return 1;
    ret = i2c_read_blocking(imu->inst, imu->addr, buff, 6, false);
    if (ret == PICO_ERROR_GENERIC) return 1;

    return 0;
}

int writeLaunchTelem(uint8_t *buff) {
    uintptr_t pe[] = {FLASH_LAUNCH_TELEM_TARGET_OFFSET, LAUNCH_TIME_TELEM_SZ};
    int ret = flash_safe_execute(flash_erase_cb, pe, 2000);
    if (ret != PICO_OK) return 1;
    uintptr_t pp[] = {FLASH_LAUNCH_TELEM_TARGET_OFFSET, LAUNCH_TIME_TELEM_SZ, (uintptr_t)buff};
    ret = flash_safe_execute(flash_write_cb, pp, 2000);
    if (ret != PICO_OK) return 1;

    return 0;
}

const uint8_t* getLaunchTelemAddr() {
    return flash_launch_telem_target;
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

    uintptr_t pe[] = {FLASH_CALIB_TARGET_OFFSET, FLASH_SECTOR_SIZE};
    int rc = flash_safe_execute(flash_erase_cb, pe, 400);
    if (rc != PICO_OK) return 2;

    uintptr_t pp[] = {FLASH_CALIB_TARGET_OFFSET, FLASH_PAGE_SIZE, (uintptr_t)buff};
    rc = flash_safe_execute(flash_write_cb, pp, 400);
    if (rc == PICO_OK) return 2;

    if (DEBUG_IMU) {
        for (int i = 0; i < 22; i++) {
            printf("0x%02X ", flash_calib_target[i]);
        }
        printf("\r\n");
    }

    WRITE_REG(OPR_MODE, imu->primary_mode)
    sleep_ms(100);

    return 0;
}

int loadCalibration(BNO055 *imu) {
    WRITE_REG(OPR_MODE, CONFIG_MODE)
    sleep_ms(100);

    uint8_t buff[22] = {0};
    memcpy(&(buff[0]), flash_calib_target, 22);

    if (DEBUG_IMU) {
        for (int i = 0; i < 22; i++) {
            printf("0x%02X ", flash_calib_target[i]);
        }
        printf("\r\n");
    }

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

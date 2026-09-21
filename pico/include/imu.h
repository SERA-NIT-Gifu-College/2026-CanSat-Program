#ifndef __IMU_H__
#define __IMU_H__

/**
 * @file imu.h
 * @brief IMU controlling functions
 */

#include <hardware/i2c.h>

/** Union for spliting values for AXIS_MAP_CONFIG and AXIS_MAP_SIGN */
typedef union {
    uint16_t cfg;
    uint8_t config[2];
} BNO055_Axis_CFG;

/** List of available I2C registers. Refer Bosch BNO055 datasheet for details. */
typedef enum {
    PAGE_ID = 0x07,

    // Page 0
    ACC_DATA_START = 0x08,
    MAG_DATA_START = 0x0E,
    GYR_DATA_START = 0x14,
    EUL_DATA_START = 0x1A,
    QUA_DATA_START = 0x20,
    LIA_DATA_START = 0x28,
    GRV_DATA_START = 0x2E,
    TEMP = 0x34,
    CALIB_STAT,
    ST_RESULT,
    INT_STA,
    SYS_CLK_STATUS,
    SYS_STATUS,
    SYS_ERR,
    UNIT_SEL,
    OPR_MODE = 0x3D,
    PWR_MODE,
    SYS_TRIGGER,
    TEMP_SOURCE,
    AXIS_MAP_CONFIG,
    AXIS_MAP_SIGN,
    SIC_MATRIX_START = 0x43,
    CALIB_DATA_START = 0x55,

    // Page 1
    ACC_CONFIG = 0x08,
    MAG_CONFIG,
    GYR_CONFIG_0,
    GYR_CONFIG_1,
    ACC_SLEEP_CONFIG,
    GYR_SLEEP_CONFIG,
    INT_MSK = 0x0F,
    INT_EN,
    ACC_AM_THRES,
    ACC_INT_SETTINGS,
    ACC_HG_DURATION,
    ACC_HG_THRES,
    ACC_NM_THRES,
    ACC_NM_SET,
    GYR_INT_SETTINGS,
    GYR_HR_X_SET,
    GYR_DUR_X,
    GYR_HR_Y_SET,
    GYR_DUR_Y,
    GYR_HR_Z_SET,
    GYR_DUR_Z,
    GYR_AM_THRES,
    GYR_AM_SET
} BNO055_Registers;

/** Operation Modes for BNO055 */
typedef enum {
    CONFIG_MODE,
    ACCONLY,
    MAGONLY,
    GYROONLY,
    ACCMAG,
    ACCGYRO,
    MAGGYRO,
    AMG,
    IMU,
    COMPASS,
    M4G,
    NDOF_FMC_OFF,
    NDOF
} BNO055_Operation_Modes;

/** Axis configurations listed in BNO055 datasheet @see BNO055_Axis_CFG */
typedef enum _ : uint16_t {
  P0 = 0x2104,
  P1 = 0x2400,
  P2 = 0x2406,
  P3 = 0x2102,
  P4 = 0x2403,
  P5 = 0x2101,
  P6 = 0x2107,
  P7 = 0x2405
} BNO055_Axis_Presets;

/** Device descriptior for BNO055 */
typedef struct {
    i2c_inst_t *inst;
    uint8_t addr;
    BNO055_Operation_Modes primary_mode;
} BNO055;

/**
 * Setup IMU from provided device descriptor
 * @param imu a pointer to a device descriptor; inst, addr, and primary_mode fields should be filled beforehand
 * @return an error code; 0 for success, 1 for NAK
 */
int setupIMU(BNO055 *imu);
/**
 * Save calibration data
 * @param imu a pointer to a device descriptor
 * @param dest a pointer to a 22 byte long array, which calibration data will be saved to
 * @return an error code; 0 for success, 1 for NAK
 */
int saveCalibration(BNO055 *imu, uint8_t *dest);
/**
 * Load calibration data
 * @param imu a pointer to a device descriptor
 * @param src a pointer to a 22 byte long array to be loaded from as pre-calibrated data
 * @return an error code; 0 for success, 1 for NAK
 */
int loadCalibration(BNO055 *imu, uint8_t *src);
/**
 * Read IMU's orientation as euler angles
 * @param imu a pointer to a device descriptor
 * @param euler an array of three integers representing euler angle as raw bytes
 * @return an error code; 0 for success, 1 for NAK
 */
int readIMU(BNO055 *imu, uint16_t euler[3]);
/**
 * Obtain IMU status information
 * @param imu a pointer to a device descriptor
 * @param calib a pointer to a byte sized variable storing calibration status
 * @param ssc a pointer to a byte sized variable storing system status code
 * @param ser a pointer to a byte sized variable storing system error code
 * @return an error code; 0 for success, 1 for NAK
 */
int obtainStatus(BNO055 *imu, uint8_t *calib, uint8_t *ssc, uint8_t *ser);

#endif // __IMU_H__

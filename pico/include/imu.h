#ifndef __IMU_H__
#define __IMU_H__

#include <hardware/i2c.h>

typedef union {
    uint16_t cfg;
    uint8_t config[2];
} BNO055_Axis_CFG;

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

typedef struct {
    i2c_inst_t *inst;
    uint8_t addr;
    BNO055_Operation_Modes primary_mode;
    int16_t euler[3];
} BNO055;

void setupIMU(BNO055 *imu);
void saveCalibration(BNO055 *imu, uint8_t *dest);
void loadCalibration(BNO055 *imu, uint8_t *src);
void readIMU(BNO055 *imu);
void obtainStatus(BNO055 *imu, uint8_t *calib, uint8_t *ssc, uint8_t *ser);

#endif // __IMU_H__

#ifndef __CHIP_ENABLE_H__
#define __CHIP_ENABLE_H__

/**
 * @file chip_enable.h
 * @brief Control codes for chip enable signals.
 */

#include <stdbool.h>

#define MOTOR_EN_PIN 2
#define WPT_EN_PIN 3
#define IMU_RST_PIN 4

void enableChips(bool motor, bool wpt, bool imu);

#endif // __CHIP_ENABLE_H__

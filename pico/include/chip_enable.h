#ifndef __CHIP_ENABLE_H__
#define __CHIP_ENABLE_H__

/**
 * @file chip_enable.h
 * @brief Control codes for chip enable signals.
 */

#include <stdbool.h>

#define MOTOR_EN_PIN 14     //!< Pin number for motor driver/V_m boost converter enable signal
#define WPT_EN_PIN 15       //!< Pin number for high-side switch signal of wireless power transmitter module
#define IMU_RST_PIN 4       //!< Pin number for BNO055's nRESET signal

/**
 * Sets enable signals for each chips
 * @param motor - enable status for motor driver/V_m boost converter
 * @param wpt - enable status for wireless power transmitter module
 * @param imu - enable status for IMU
 */
void enableChips(bool motor, bool wpt, bool imu);

#endif // __CHIP_ENABLE_H__

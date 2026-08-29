#ifndef __CHIP_ENABLE_H__
#define __CHIP_ENABLE_H__

/**
 * @file chip_enable.h
 * @brief Definition of bits for chip enable signals.
 */

#define CHIP_ENABLE_INIT_MODE (0x00) //!< Chip enable preset for initial power on, all chip disabled.
#define CHIP_ENABLE_LIPO_MODE (CHIP_ENABLE_MOTOR_5V | CHIP_ENABLE_IMU | CHIP_ENABLE_LIGHT_SENSOR) //!< Chip enable preset for normal Lipo battery power, enables motor drivers, motor power, IMU, and a light sensor.
#define CHIP_ENABLE_SOLAR_SELF_CHARGE_MODE (CHIP_ENABLE_SOLAR_5V | CHIP_ENABLE_CHARGER) //!< Chip enable preset for Solar Lipo battery charge, enables solar boost converter and charger IC. The power from battery will be cut off.
#define CHIP_ENABLE_SOLAR_WPT_MODE (CHIP_ENABLE_SOLAR_5V | CHIP_ENABLE_WIRELESS_POWER_TRANSMITTER) //!< Chip enable preset for solar wireless power transmittion, enables solar boost converter and transmitter. The power from battery will be cut off.

/**
 * Bit positions for each enable signal
 */
typedef enum {
    CHIP_ENABLE_SOLAR_5V                    = 1 << 0,
    CHIP_ENABLE_MOTOR_5V                    = 1 << 1,
    CHIP_ENABLE_CHARGER                     = 1 << 2,
    CHIP_ENABLE_WIRELESS_POWER_TRANSMITTER  = 1 << 3,
    CHIP_ENABLE_IMU                         = 1 << 4,
    CHIP_ENABLE_LIGHT_SENSOR                = 1 << 5,
} Chips;

#endif // __CHIP_ENABLE_H__

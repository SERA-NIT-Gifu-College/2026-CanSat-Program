#ifndef __MOTOR_H__
#define __MOTOR_H__

/**
 * @file motor.h
 * @brief Helper functions for controlling onboard motors
 */

#include <stdint.h>

#define GAIN_P 1.0f                 //!< Proportional gain for wheel drive
#define GAIN_I 1.0f                 //!< Integral gain for wheel drive
#define GAIN_D 1.0f                 //!< Derivative gain for wheel drive

#define TARGET_ANGLE_MARGIN 5.0f    //!< Angle margin for wheel drive

/** Wheel motors descriptor */
typedef struct {
    uint8_t slice_m1;
    uint8_t chA_m1;
    uint8_t chB_m1;
    uint8_t slice_m2;
    uint8_t chA_m2;
    uint8_t chB_m2;
    uint16_t target;
    uint16_t current;
} Wheels;

/** Cycle controlled servo motor descriptor */
typedef struct {
    uint8_t slice;
    uint8_t ch;
    uint16_t cycles;
    uint16_t c_base;
    uint16_t wrap_val;
} Servo;

void initWheels(Wheels *w);
void setTargetAngle(Wheels *w, float d);
int runWheels(Wheels *w, uint16_t *angle_sensor_value);

void initServo(Servo *s, float f_div, uint16_t t_min, uint16_t t_max);
int setOnCycles(Servo *s, uint16_t c);

#endif // __MOTOR_H__

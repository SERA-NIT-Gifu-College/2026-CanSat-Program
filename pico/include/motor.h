#ifndef __MOTOR_H__
#define __MOTOR_H__

/**
 * @file motor.h
 * @brief Helper functions for controlling onboard motors
 */

#include <stdint.h>
#include <stdbool.h>

#define GAIN_P (1.0f)               //!< Proportional gain for wheel drive
#define GAIN_I (1.0f)               //!< Integral gain for wheel drive
#define GAIN_D (1.0f)               //!< Derivative gain for wheel drive

#define TARGET_ANGLE_MARGIN (2.0f)  //!< +/- Angle margin for wheel drive

#define ANGLE_RESOLUTION (8.0f)     //!< Angle resolution for servo motor; minimum angle difference will be inverse of this value

typedef enum {
    STRAIGHT,
    INDEPENDENT,
    ABS_ANGLE,
    DELTA_ANGLE,
    NUM_MOVE_MODE
} MoveMode;

/** PID controller descriptor */
typedef struct {
    uint16_t target;
    uint16_t current;
    int64_t time_prev;
    int64_t time_curr;
    double p;
    double i;
    double d;
    bool onTarget;
} PIDController;

/** Wheel motors descriptor */
typedef struct {
    uint8_t slice_m1;
    uint8_t slice_m2;
    bool running_1;
    bool running_2;
    PIDController *pid;
} Wheels;

/** Cycle controlled servo motor descriptor */
typedef struct {
    uint8_t slice;
    uint8_t ch;
    uint16_t c_base;
    uint16_t wrap_val;
    float angle;
} Servo;

void initWheels(Wheels *w, PIDController *pid);
int setTargetAngle(Wheels *w, uint16_t d);
int driveWheelsForMs(Wheels *w, int8_t lev1, int8_t lev2, uint16_t t1, uint16_t t2);
int runWheels(Wheels *w, uint16_t imu_heading);

void initServo(Servo *s, float freq, uint16_t t_min, uint16_t t_max);
int setServoAngle(Servo *s, float ang);

#endif // __MOTOR_H__

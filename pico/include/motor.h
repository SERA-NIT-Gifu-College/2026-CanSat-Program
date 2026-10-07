#ifndef __MOTOR_H__
#define __MOTOR_H__

/**
 * @file motor.h
 * @brief Helper functions for controlling onboard motors
 */

#include <stdint.h>
#include <stdbool.h>
#include <hardware/timer.h>

#ifndef USE_VARIABLE_GAIN
#define GAIN_P (1.0f)               //!< Proportional gain for wheel drive
#define GAIN_I (1.0f)               //!< Integral gain for wheel drive
#define GAIN_D (1.0f)               //!< Derivative gain for wheel drive
#endif

#define TARGET_ANGLE_MARGIN (2.0f)  //!< +/- Angle margin for wheel drive

#define ANGLE_RESOLUTION (8.0f)     //!< Angle resolution for servo motor; minimum angle difference will be inverse of this value

#define PID_TIME_KEEPING_TIMER_HW timer1_hw

/** Move modes */
typedef enum {
    STRAIGHT,
    INDEPENDENT,
    ABS_ANGLE,
    DELTA_ANGLE,
    NUM_MOVE_MODE
} MoveMode;

/** PID controller descriptor */
typedef struct {
    double target;
    double current;
    uint64_t time_prev;
    uint64_t time_curr;
    double prev_err;
    double int_err;
    bool onTarget;
#ifdef USE_VARIABLE_GAIN
    double gain[3];
#endif // USE_VARIABLE_GAIN
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

/**
 * Initialize wheels' PWM and descriptor
 * @param w a pointer to wheels descriptor
 * @param pid a pointer to PID controller descriptor
 */
void initWheels(Wheels *w, PIDController *pid);
/**
 * Sets and start angle/heading control
 * @param w a pointer to wheels descriptor
 * @param d target angle in degree
 * @return 0 for success, 1 for rejected request
 */
int setTargetAngle(Wheels *w, uint16_t d);
/**
 * Spins wheels for certain period at certain speed
 * @param w a pointer to wheels descriptor
 * @param lev1 PWM level for first wheel. Set this value negative to spin backward. Range is -100 ~ 100.
 * @param lev2 PWM level for second wheel. Set this value negative to spin backward. Range is -100 ~ 100.
 * @param t1 Spin duration of first wheel in milliseconds
 * @param t2 Spin duration of second wheel in milliseconds
 * @return 0 for success, 1 for rejected request
 */
int driveWheelsForMs(Wheels *w, int8_t lev1, int8_t lev2, uint16_t t1, uint16_t t2);
/**
 * PID controller loop function for wheels; runs single "tick" of control
 * @param w a pointer to wheels descriptor
 * @param imu_heading IMU reading of heading in degree
 * @return 0 for success tick, 1 for rejected tick
 */
int wheelsPIDTick(Wheels *w, double imu_heading);

/**
 * Initialize servo with specified frequency, min and max pulse width
 * @param s a pointer to servo motor descriptor
 * @param freq frequency of PWM
 * @param t_min minimum pulse width
 * @param t_max maximum pulse width
 */
void initServo(Servo *s, float freq, uint16_t t_min, uint16_t t_max);
/**
 * Sets servo at specified "angle"
 * @param s a pointer to servo motor descriptor
 * @param ang "angle" of servo; range will be 0 ~ 360 regardless a type of servo
 * @return 0 for success, 1 for incorrect ang parameter
 */
int setServoAngle(Servo *s, float ang);

#endif // __MOTOR_H__

#include "motor.h"

#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <pico/stdlib.h>
#include <stdint.h>

#define DEBUG_MOTOR
#define MOTOR_DRY_RUN

#ifdef DEBUG_MOTOR
#include <stdio.h>
#endif

static int64_t wheel_alarm_cb_1(alarm_id_t _id, void *data) {
    Wheels *w = (Wheels*)data;

    pwm_set_both_levels(w->slice_m1, 0, 0);

    w->running_1 = false;

#ifdef DEBUG_MOTOR
    printf("wheel_alarm_cb_1 fired: id=%d\r\n", _id);
#endif

    return 0;
}
static int64_t wheel_alarm_cb_2(alarm_id_t _id, void *data) {
    Wheels *w = (Wheels*)data;

    pwm_set_both_levels(w->slice_m2, 0, 0);

    w->running_2 = false;

#ifdef DEBUG_MOTOR
    printf("wheel_alarm_cb_2 fired: id=%d\r\n", _id);
#endif

    return 0;
}

void initWheels(Wheels *w) {
    const float wrap_v = 10000.f;

    w->running_1 = false;
    w->running_2 = false;
    pwm_config c = pwm_get_default_config();

    pwm_config_set_clkdiv(&c, ((float)clock_get_hz(clk_sys) / wrap_v) / (1.0f * 1000.f));
    pwm_config_set_wrap(&c, wrap_v - 1);

    pwm_init(w->slice_m1, &c, false);
    pwm_init(w->slice_m2, &c, false);

    pwm_set_both_levels(w->slice_m1, 0, 0);
    pwm_set_both_levels(w->slice_m2, 0, 0);
}

int driveWheelsForMs(Wheels *w, int8_t lev1, int8_t lev2, uint16_t t1, uint16_t t2) {
    if (w->running_1 || w->running_2) return 1;
    w->running_1 = true;
    w->running_2 = true;
    pwm_set_enabled(w->slice_m1, false);
    pwm_set_enabled(w->slice_m2, false);
    sleep_ms(10);
    if (lev1 < 0) {
        pwm_set_both_levels(w->slice_m1, 0, -lev1 * 100);
    } else {
        pwm_set_both_levels(w->slice_m1, lev1 * 100, 0);
    }
    if (lev2 < 0) {
        pwm_set_both_levels(w->slice_m2, 0, -lev2 * 100);
    } else {
        pwm_set_both_levels(w->slice_m2, lev2 * 100, 0);
    }
    sleep_ms(10);
    pwm_set_enabled(w->slice_m1, true);
    pwm_set_enabled(w->slice_m2, true);
    add_alarm_in_ms(t1, wheel_alarm_cb_1, (void*)w, true);
    add_alarm_in_ms(t2, wheel_alarm_cb_2, (void*)w, true);

    return 0;
}

int setTargetAngle(Wheels *w, uint16_t angle) {
    if (w->running_1 || w->running_2) {
        printf("either motor is running, rejected\r\n");
        return 1;
    }
    w->running_1 = true;
    w->running_2 = true;
    w->target = angle;
    return 0;
}

void initServo(Servo *s, float freq, uint16_t t_min, uint16_t t_max) {
    float t = 1.f/freq;
    const float u = 1.f/(1000.f * 1000.f);
    // In order to achive < 100 Hz on PWM, TOP(wrap_value) needs to be large
    // enough, thats where ANGLE_RESOLITION multiplier comes in with
    // a side effect of being able to set fractional angle.
    uint16_t wrap_value = (uint16_t)(ANGLE_RESOLUTION * 360.f * (t/((t_max - t_min) * u)));
    uint16_t cycle_base = (uint16_t)((float)wrap_value * ((float)t_min * u)/t);
    s->c_base = cycle_base;
    s->wrap_val = wrap_value;
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, ((float)clock_get_hz(clk_sys) / (float)wrap_value) / freq);
    pwm_config_set_wrap(&c, wrap_value - 1);
    pwm_init(s->slice, &c, true);
    setServoAngle(s, 0);
}

int setServoAngle(Servo *s, float ang) {
    if (ang > 360 || ang < 0) return 1;
    s->angle = ang;
    pwm_set_chan_level(s->slice, s->ch, (uint16_t)(s->c_base + ANGLE_RESOLUTION*s->angle));
    return 0;
}

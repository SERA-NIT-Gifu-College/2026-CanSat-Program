#include "motor.h"

#include <hardware/pwm.h>
#include <stdint.h>

void initServo(Servo *s, float f_div, uint16_t t_min, uint16_t t_max) {
    float t = f_div/(125.f * 1000.f * 1000.f);
    const float u = 1/(1000 * 1000);
    uint16_t wrap_value = (uint16_t)(360.f * (t/((t_max - t_min) * u)));
    uint16_t cycle_base = (uint16_t)((float)wrap_value * ((float)t_min * u)/t);
    s->c_base = cycle_base;
    s->wrap_val = wrap_value;
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, f_div);
    pwm_config_set_wrap(&c, wrap_value - 1);
    pwm_init(s->slice, &c, true);
}

int setOnCycles(Servo *s, uint16_t c) {
    if (c > 360) return 1;
    s->cycles = c;
    pwm_set_chan_level(s->slice, s->ch, s->c_base + s->cycles);
    return 0;
}

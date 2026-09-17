#include "motor.h"

#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <stdint.h>

void initServo(Servo *s, float freq, uint16_t t_min, uint16_t t_max) {
    float t = 1.f/freq;
    const float u = 1.f/(1000.f * 1000.f);
    uint16_t wrap_value = (uint16_t)(ANGLE_RESOLUTION * 360.f * (t/((t_max - t_min) * u)));
    uint16_t cycle_base = (uint16_t)((float)wrap_value * ((float)t_min * u)/t);
    s->c_base = cycle_base;
    s->wrap_val = wrap_value;
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, ((float)clock_get_hz(clk_sys) / (float)wrap_value) / freq);
    pwm_config_set_wrap(&c, wrap_value - 1);
    pwm_init(s->slice, &c, true);
    setOnCycles(s, 0);
}

int setOnCycles(Servo *s, float c) {
    if (c > 360) return 1;
    s->cycles = c;
    pwm_set_chan_level(s->slice, s->ch, (uint16_t)(s->c_base + ANGLE_RESOLUTION*s->cycles));
    return 0;
}

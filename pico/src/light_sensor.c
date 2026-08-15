#include "light_sensor.h"
#include <pico/stdlib.h>

void setupLightSensor() {
    adc_init();
    adc_gpio_init(26 + LIGHT_SENSOR_ADC_NUM);
}

uint16_t readAVG() {
    uint32_t s = 0;
    adc_select_input(LIGHT_SENSOR_ADC_NUM);
    for (int i = 0; i < LIGHT_SENSOR_SAMPLING_NUMBER; i++) {
        s += adc_read();
        sleep_ms(LIGHT_SENSOR_SAMPLING_INTERVAL);
    }
    return s / LIGHT_SENSOR_SAMPLING_NUMBER;
}

#include "solar.h"
#include <pico/stdlib.h>

void setupSolarMon() {
    adc_gpio_init(26 + SOLAR_ADC_NUM);
}

uint16_t getGen() {
    adc_select_input(SOLAR_ADC_NUM);
    return adc_read();
}

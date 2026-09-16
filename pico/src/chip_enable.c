#include "chip_enable.h"
#include <hardware/gpio.h>

void enableChips(bool motor, bool wpt, bool imu) {
    gpio_put(MOTOR_EN_PIN, motor ? 1 : 0);
    gpio_put(WPT_EN_PIN, wpt ? 1 : 0);
    gpio_put(IMU_RST_PIN, imu ? 1 : 0);
}


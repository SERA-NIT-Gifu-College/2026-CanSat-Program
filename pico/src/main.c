#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>

#include <stdio.h>

#include "imu.h"
#include "light_sensor.h"

int main(void) {
    stdio_init_all();

    gpio_init(7);
    gpio_set_dir(7, GPIO_OUT);

    gpio_set_function(8, GPIO_FUNC_I2C);
    gpio_set_function(9, GPIO_FUNC_I2C);
    i2c_init(i2c0, 400 * 1000);

    gpio_put(7, 0);

    BNO055 imu = {
      .inst = i2c0,
      .addr = 0x28,
      .primary_mode = NDOF
    };

    uint8_t imu_calib_stat = 0;
    uint8_t imu_sys_stat = 0;
    uint8_t imu_sys_err = 0;

    setupIMU(&imu);
    setupLightSensor();

    while (1) {
        const float conv_factor = 3.3f / (1 << 12);
        uint16_t l = readAVG();
        obtainStatus(&imu, &imu_calib_stat, &imu_sys_stat, &imu_sys_err);
        readIMU(&imu);
        printf("Result: %02X, %f V\n", l, l * conv_factor);
        printf("IMU Calib: %02X, IMU STAT: %02X, IMU ERR: %02X\n", imu_calib_stat, imu_sys_stat, imu_sys_err);
        printf("IMU Euler: %f, %f, %f\n", imu.euler[0] / 16.f, imu.euler[1] / 16.f, imu.euler[2] / 16.f);
        sleep_ms(100);
    }

    return 0;
}

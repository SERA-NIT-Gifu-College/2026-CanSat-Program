#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <stdio.h>

#include <hardware/i2c.h>
#include <hardware/gpio.h>

#include "imu.h"
#include "chip_enable.h"

BNO055 imu = {
    .inst = i2c1,
    .addr = 0x28,
    .primary_mode = NDOF
};

int main(void) {
    stdio_init_all();

    sleep_ms(5000);

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);

    gpio_init(IMU_RST_PIN);
    gpio_set_dir(IMU_RST_PIN, true);
    enableChips(false, false, true);

    sleep_ms(1000);

    BNO055_Axis_CFG axis;

    axis.config[0] = 0x12;
    axis.config[1] = 0x04;

    printf("setup imu\r\n");
    setupIMU(&imu);

    printf("remap axis\r\n");
    uint8_t buff[2] = {0};
    buff[0] = OPR_MODE; buff[1] = CONFIG_MODE;
    i2c_write_blocking(imu.inst, imu.addr, buff, 2, true);
    sleep_ms(100);
    buff[0] = AXIS_MAP_CONFIG; buff[1] = axis.config[0];
    i2c_write_blocking(imu.inst, imu.addr, buff, 2, true);
    buff[0] = AXIS_MAP_SIGN; buff[1] = axis.config[1];
    i2c_write_blocking(imu.inst, imu.addr, buff, 2, true);
    buff[0] = OPR_MODE; buff[1] = imu.primary_mode;
    i2c_write_blocking(imu.inst, imu.addr, buff, 2, true);
    sleep_ms(100);

    int16_t b[3] = {0};

    printf("start acc\r\n");
    while (1) {
        getAcc(&imu, (uint8_t*)(&b[0]));

        printf("%lf, %lf, %lf\r\n", b[0]/100.0, b[1]/100.0, b[2]/100.0);

        readIMU(&imu, (uint16_t*)(&b[0]));
        printf("%lf, %lf, %lf\r\n", b[0]/16.0, b[1]/16.0, b[2]/16.0);

        sleep_ms(100);
    }

    return 0;
}

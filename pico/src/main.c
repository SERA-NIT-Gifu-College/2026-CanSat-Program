#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/cyw43_arch.h>
#include <cyw43_ll.h>
#include <lwip/inet.h>
#include <stdio.h>

#include "imu.h"
#include "light_sensor.h"
#include "server.h"

Server server = {0};

int connectWifi(void) {
    if (cyw43_arch_init()) {
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    int res = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWD, CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (res) {
        watchdog_enable(5 * 1000, true);
        return 2;
    }

    return 0;
}

int main(void) {
    stdio_init_all();

    if (watchdog_caused_reboot()) {
        printf("Reseted by watchdog\r\n");
    }

    int conn_stat = connectWifi();
    if (conn_stat) {
        switch (conn_stat) {
            case 1:
                printf("Failed to initialize Wifi\r\n");
                break;
            case 2:
                printf("Connection timeout\r\n");
                break;
        }
        printf("Failed to Connect to Wifi, restarting...\r\n");
        while (1) ; // wait for watchdog reset
        return 1;
    }

    printf("Connected to Wifi! IP address is: %s\r\n", inet_ntoa(netif_default->ip_addr.addr));

    watchdog_enable(1000, true);

    gpio_init(7);
    gpio_set_dir(7, GPIO_OUT);

    gpio_set_function(8, GPIO_FUNC_I2C);
    gpio_set_function(9, GPIO_FUNC_I2C);
    i2c_init(i2c0, 400 * 1000);

    gpio_put(7, 0);
    sleep_ms(200);
    gpio_put(7, 1);
    sleep_ms(200);

    watchdog_update();

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

    err_t e = server_init(&server);

    if (e != ERR_OK) {
        printf("Failed to initialize server, restarting...\r\n");
        while (1) ; // wait for watchdog reset
        return 1;
    }

    while (1) {
        const float conv_factor = 3.3f / (1 << 12);
        uint16_t l = readAVG();
        obtainStatus(&imu, &imu_calib_stat, &imu_sys_stat, &imu_sys_err);
        readIMU(&imu);
        /*
        printf("Result: %02X, %f V\n", l, l * conv_factor);
        printf("IMU Calib: %02X, IMU STAT: %02X, IMU ERR: %02X\n", imu_calib_stat, imu_sys_stat, imu_sys_err);
        printf("IMU Euler: %f, %f, %f\n", imu.euler[0] / 16.f, imu.euler[1] / 16.f, imu.euler[2] / 16.f);
        */
        sleep_ms(100);
        watchdog_update();
    }

    cyw43_arch_disable_sta_mode();
    cyw43_arch_deinit();

    return 0;
}

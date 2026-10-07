#include <pico/stdlib.h>
#include <pico/cyw43_arch.h>
#include <pico/lwip_nosys.h>

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>

#include <cyw43.h>
#include <cyw43_ll.h>
#include <lwip/inet.h>

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#define USE_VARIABLE_GAIN

#include "imu.h"
#include "motor.h"
#include "server.h"
#include "chip_enable.h"

BNO055 imu = {.inst = i2c1, .addr = 0x28, .primary_mode = NDOF};
Wheels w = {0};
Server server = {0};
bool setAngle = false;

void connectWifi(void) {
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_JAPAN)) {
        printf("Failed to init wifi chip\r\n");
        return;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(500);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    cyw43_arch_enable_sta_mode();

connection:
    {
    int res = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWD, CYW43_AUTH_WPA2_AES_PSK, 15000);
    if (res) {
        printf("Failed to connect Wifi, retrying...\r\n");
        goto connection;
    }
    }

    for (char i = 0; i < 3; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(100);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(100);
    }
}

void callback(void *arg) {
    Server *srv = (Server*)arg;
    for (int i = 0; i < TX_BUFF_SZ; i++) {
        srv->tx_buff[i] = 0;
    }

    uint8_t buff[TX_BUFF_SZ] = {0};

    switch (srv->cmd) {
        case PING:
            memcpy(srv->tx_buff, "pong\r\n", PING_PACKET_TX_LEN);
            break;
        case GET_TELEMETRY:
            readIMU(&imu, (uint16_t*)(&buff[0]));
            obtainStatus(&imu, &buff[12], &buff[10], &buff[11]);
            memcpy(srv->tx_buff, buff, TX_BUFF_SZ);
            break;
        case MOVE:
            {
            MoveMode mode = srv->rx_buff[1];
            switch (mode) {
                case STRAIGHT:
                    {
                    int8_t lev = (int8_t)(srv->rx_buff[2]);
                    uint16_t t = ((uint16_t)(srv->rx_buff[3]) << 8) | (srv->rx_buff[4]);
                    int res = driveWheelsForMs(&w, lev, lev, t, t);
                    buff[0] = res;
                    memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                    break;
                    }
                case INDEPENDENT:
                    {
                    int8_t lev1 = (int8_t)(srv->rx_buff[2]);
                    uint16_t t1 = ((uint16_t)(srv->rx_buff[3]) << 8) | (srv->rx_buff[4]);
                    int8_t lev2 = (int8_t)(srv->rx_buff[5]);
                    uint16_t t2 = ((uint16_t)(srv->rx_buff[6]) << 8) | (srv->rx_buff[7]);
                    int res = driveWheelsForMs(&w, lev1, lev2, t1, t2);
                    buff[0] = res;
                    memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                    break;
                    }
                case ABS_ANGLE:
                    {
                    uint16_t angle = ((uint16_t)(srv->rx_buff[2]) << 8) | (srv->rx_buff[3]);
                    int res = setTargetAngle(&w, angle);
                    setAngle = true;
                    buff[0] = res;
                    memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                    break;
                    }
                case DELTA_ANGLE:
                    {
                    int16_t angle = (int16_t)(((uint16_t)(srv->rx_buff[2]) << 8) | (srv->rx_buff[3]));
                    angle = w.target + angle;
                    if (angle < 0) {
                        angle = 360 + angle;
                    }
                    int res = setTargetAngle(&w, angle);
                    setAngle = true;
                    buff[0] = res;
                    memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                    break;
                    }
                default:
                    break;
            };
            break;
            }
        default:
            break;
    }
}

int main(void) {
    stdio_init_all();

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    sleep_ms(500);
    gpio_init_mask((1 << MOTOR_EN_PIN) | (1 << WPT_EN_PIN) | (1 << IMU_RST_PIN));
    gpio_set_dir_out_masked((1 << MOTOR_EN_PIN) | (1 << WPT_EN_PIN) | (1 << IMU_RST_PIN));
    gpio_pull_up(IMU_RST_PIN);
    enableChips(true, false, true);

    connectWifi();
    printf("IP: %s\r\n", inet_ntoa(netif_default->ip_addr.addr));

    printf("Setting up IMU...\r\n");
    setupIMU(&imu);

    printf("Setting up wheels...\r\n");
    gpio_set_function(8, GPIO_FUNC_PWM);
    gpio_set_function(9, GPIO_FUNC_PWM);
    gpio_set_function(10, GPIO_FUNC_PWM);
    gpio_set_function(11, GPIO_FUNC_PWM);
    w.slice_m1 = pwm_gpio_to_slice_num(8);
    w.slice_m2 = pwm_gpio_to_slice_num(10);
    initWheels(&w);

    printf("Setting up server...\r\n");
    server_init(&server, callback);

    printf("Complete!\r\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    uint16_t b[3] = {0};

    while (1) {
        if (setAngle) {
            readIMU(&imu, b);
            double current = (int16_t)b[0] / 16.0;
            double e = w.target - current;
            bool rotDir = e > 0.0;
            if ((e < 0.0 ? -e : e) > TARGET_ANGLE_MARGIN) {
                pwm_set_both_levels(w.slice_m1,
                    rotDir ? 5000 : 0,
                    rotDir ? 0 : 5000
                );
                pwm_set_both_levels(w.slice_m2,
                    rotDir ? 0 : 5000,
                    rotDir ? 5000 : 0
                );
            } else {
                w.running_1 = false;
                w.running_2 = false;
                setAngle = false;
                pwm_set_both_levels(w.slice_m1, 0, 0);
                pwm_set_both_levels(w.slice_m2, 0, 0);
            }
        }
        sleep_ms(10);
    }

    return 0;
}

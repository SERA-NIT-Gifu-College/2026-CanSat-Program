#include <cyw43.h>
#include <hardware/adc.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <hardware/pio.h>
#include <hardware/sync.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/cyw43_arch.h>
#include <pico/lwip_nosys.h>
#include <cyw43_ll.h>
#include <lwip/inet.h>
#include <stdbool.h>
#include <stdio.h>

#include "imu.h"
#include "light_sensor.h"
#include "server.h"
#include "solar.h"
#include "chip_enable.h"

Server server = {0};
Telemetry telem = {0};
BNO055 imu = {
    .inst = i2c1,
    .addr = 0x28,
    .primary_mode = NDOF
};

int connectWifi(void) {
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_JAPAN)) {
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    int res = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWD, CYW43_AUTH_WPA2_AES_PSK, 15000);
    if (res) {
        watchdog_enable(5 * 1000, true);
        return 2;
    }

    res = cyw43_wifi_pm(&cyw43_state, CYW43_AGGRESSIVE_PM);
    if (res != 0) {
        watchdog_enable(3 * 1000, true);
        return 3;
    }

    return 0;
}

void serializeTelemetry(Telemetry *t, uint8_t *b) {
    b[0] = (uint8_t)(t->euler_raw[0] & 0x00FF);
    b[1] = (uint8_t)((t->euler_raw[0] & 0xFF00) >> 8);
    b[2] = (uint8_t)(t->euler_raw[1] & 0x00FF);
    b[3] = (uint8_t)((t->euler_raw[1] & 0xFF00) >> 8);
    b[4] = (uint8_t)(t->euler_raw[2] & 0x00FF);
    b[5] = (uint8_t)((t->euler_raw[2] & 0xFF00) >> 8);
    
    b[6] = (uint8_t)(t->light_sensor_raw & 0x00FF);
    b[7] = (uint8_t)((t->light_sensor_raw & 0xFF00) >> 8);

    b[8] = (uint8_t)(t->solar_cell_raw & 0x00FF);
    b[9] = (uint8_t)((t->solar_cell_raw & 0xFF00) >> 8);

    b[10] = t->imu_status;
    b[11] = t->imu_error;
    b[12] = t->imu_calib;

    b[13] = 'N';
    b[14] = 'I';
    b[15] = 'T';
}

void callback(void *arg) {
    Server *srv = (Server*)arg;

    for (int i = 0; i < TX_BUFF_SZ; i++) {
        srv->tx_buff[i] = 0;
    }

    switch (srv->cmd) {
        case GET_TELEMETRY:
            readIMU(&imu, telem.euler_raw);
            obtainStatus(
                &imu,
                &(telem.imu_calib),
                &(telem.imu_status),
                &(telem.imu_error)
            );
            telem.light_sensor_raw = readAVG();
            telem.solar_cell_raw = getGen();
            sleep_ms(200);
            uint8_t buff[TX_BUFF_SZ] = {0};
            serializeTelemetry(&telem, buff);
            memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
            break;
        case PING:
            memcpy(&(srv->tx_buff[0]), "pong\r\n", 6);
            break;
        default:
            break;
    }
}

int main(void) {
    stdio_init_all();

    gpio_init_mask((1 << MOTOR_EN_PIN) | (1 << WPT_EN_PIN) | (1 << IMU_RST_PIN));
    enableChips(false, false, true);

    sleep_ms(5000);

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
            case 3:
                printf("Failed to set wifi power management\r\n");
                break;
        }
        printf("Failed to Connect to Wifi, restarting...\r\n");
        while (1) ; // wait for watchdog reset
        return 1;
    }

    printf("Connected to Wifi! IP address is: %s\r\n", inet_ntoa(netif_default->ip_addr.addr));

    watchdog_enable(1000, true);

    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    i2c_init(i2c1, 400 * 1000);

    watchdog_update();

    setupIMU(&imu);

    adc_init();
    setupLightSensor();
    setupSolarMon();

    err_t e = server_init(&server, callback);

    if (e != ERR_OK) {
        printf("Failed to initialize server, restarting...\r\n");
        while (1) ; // wait for watchdog reset
        return 1;
    }

    while (1) {
        sleep_ms(50);
        watchdog_update();
    }

    return 0;
}

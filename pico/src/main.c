#include <cyw43.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <hardware/pio.h>
#include <hardware/powman.h>
#include <hardware/sync.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/cyw43_arch.h>
#include <pico/aon_timer.h>
#include <pico/low_power.h>
#include <cyw43_ll.h>
#include <lwip/inet.h>
#include <stdio.h>

#include "imu.h"
#include "light_sensor.h"
#include "server.h"
#include "chip_enable.h"
#include "pio_74hc595.pio.h"

Server server = {0};
BNO055 imu = {
    .inst = i2c0,
    .addr = 0x28,
    .primary_mode = NDOF
};
ShiftRegister sr = {
    .pio = pio0,
    .sm = 0,
    .ofst = 0,
    .bp = 2
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
        watchdog_enable(5 * 1000, true);
        return 3;
    }

    return 0;
}

int go_sleep(uint32_t ms, uint8_t ce) {
    uint32_t ints = save_and_disable_interrupts();

    server_close(&server);
    shiftRegister_put(&sr, ce);
    cyw43_arch_disable_sta_mode();
    cyw43_arch_deinit();

    powman_set_debug_power_request_ignored(true);
    powman_timer_disable_alarm();
    powman_clear_alarm();
    powman_timer_set_1khz_tick_source_lposc();
    powman_timer_set_ms(0);

    powman_timer_start();
    powman_enable_alarm_wakeup_at_ms(ms);

    powman_power_state state = powman_get_power_state();
    state = powman_power_state_with_domain_off(state, POWMAN_POWER_DOMAIN_SRAM_BANK0);
    state = powman_power_state_with_domain_off(state, POWMAN_POWER_DOMAIN_SRAM_BANK1);
    state = powman_power_state_with_domain_off(state, POWMAN_POWER_DOMAIN_SWITCHED_CORE);
    state = powman_power_state_with_domain_off(state, POWMAN_POWER_DOMAIN_XIP_CACHE);
    
    int res = powman_set_power_state(state);

    if (res == PICO_OK) {
        __asm volatile ("dsb sy");
        __asm volatile ("wfi");
        return 0;
    }

    restore_interrupts(ints);
    return 1;
}

void callback(void *arg) {
    Server *srv = (Server*)arg;

    for (int i = 0; i < TX_BUFF_SZ; i++) {
        srv->tx_buff[i] = 0;
    }

    int rc = 0;

    switch (srv->cmd) {
        case READ_IMU:
            readIMU(&imu);
            memcpy(srv->tx_buff, imu.euler, 6);
            break;
        case GET_IMU_STATUS:
            obtainStatus(&imu, &(srv->tx_buff[0]), &(srv->tx_buff[1]), &(srv->tx_buff[2]));
            break;
        case START_SELF_CHARGE:
            printf("Entering Self Charge Mode\r\n");
            rc = go_sleep(10*1000, CHIP_ENABLE_SOLAR_SELF_CHARGE_MODE);
            if (rc) {
                printf("Failed to enter dormant sleep, restarting with watchdog...\r\n");
                while (1) ;
            }
            break;
        case START_POWER_TRANS:
            printf("Entering WPT Mode\r\n");
            rc = go_sleep(10*1000, CHIP_ENABLE_SOLAR_WPT_MODE);
            if (rc) {
                printf("Failed to enter dormant sleep, restarting with watchdog...\r\n");
                while (1) ;
            }
            break;
        case PING:
            memcpy(srv->tx_buff, "pong\r\n", 6);
            break;
        default:
            break;
    }
}

int main(void) {
    stdio_init_all();

    pio_claim_free_sm_and_add_program(&pio_74hc595_program, &(sr.pio), &(sr.sm), &(sr.ofst));
    pio_74hc595_program_init(&sr);
    pio_sm_set_enabled(sr.pio, sr.sm, true);
    shiftRegister_put(&sr, CHIP_ENABLE_INIT_MODE);

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

    setupIMU(&imu);
    setupLightSensor();

    err_t e = server_init(&server, callback);

    if (e != ERR_OK) {
        printf("Failed to initialize server, restarting...\r\n");
        while (1) ; // wait for watchdog reset
        return 1;
    }

    while (1) {
        sleep_ms(10);
        watchdog_update();
    }

    cyw43_arch_disable_sta_mode();
    cyw43_arch_deinit();

    return 0;
}

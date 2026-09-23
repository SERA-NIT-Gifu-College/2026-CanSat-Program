#include <hardware/adc.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <hardware/pwm.h>
#include <hardware/sync.h>

#include <pico/cyw43_arch.h>
#include <pico/lwip_nosys.h>
#include <cyw43.h>
#include <cyw43_ll.h>
#include <lwip/inet.h>

#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <stdbool.h>
#include <stdio.h>

#include "imu.h"
#include "light_sensor.h"
#include "motor.h"
#include "server.h"
#include "solar.h"
#include "chip_enable.h"

bool motor_stat = false;
bool wpt_stat = false;
bool imu_stat = false;
Server server = {0};
Telemetry telem = {0};
BNO055 imu = {
    .inst = i2c1,
    .addr = 0x28,
    .primary_mode = COMPASS
};
Servo pannel = {0};
Servo parachute = {0};
PIDController pid = {0};
Wheels wheels = {0};
SunSearchSM sssm = {0};

int connectWifi(void) {
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_JAPAN)) {
        return 1;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(500);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

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

    for (char i = 0; i < 3; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(100);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(100);
    }

    return 0;
}

void sunSearch() {
    switch (sssm.state) {
        case SUNSEARCH_IDLE:
            break;
        case SUNSEARCH_SCAN_HEADING:
            if (wheels.pid->onTarget && sssm.brightness_read && !(sssm.transition)) {
                sssm.brightness_read = false;
                sssm.transition = true;
                setTargetAngle(&wheels, sssm.heading);
                sssm.heading += 45;
                break;
            } else if (wheels.pid->onTarget && !(sssm.brightness_read) && sssm.transition) {
                uint16_t b = readAVG();
                if (sssm.brightness < b) {
                    sssm.brightness = b;
                    sssm.max_brightness_heading = sssm.heading;
                }
                sssm.brightness_read = true;
                sssm.transition = true;
                if (sssm.heading == 315) {
                    sssm.heading = 0;
                    sssm.brightness = 0;
                    sssm.state = SUNSEARCH_ROTATE_BRIGHTEST;
                };
                break;
            } else {
                break;
            }
        case SUNSEARCH_ROTATE_BRIGHTEST:
            if (wheels.pid->onTarget && !(sssm.transition)) {
                setTargetAngle(&wheels, sssm.max_brightness_heading);
                sssm.transition = true;
            } else if (wheels.pid->onTarget && sssm.transition) {
                sssm.state = SUNSEARCH_SCAN_PITCH;
                sssm.transition = false;
                break;
            } else {
                break;
            }
        case SUNSEARCH_SCAN_PITCH:
            sssm.state = SUNSEARCH_SET_PITCH;
            break;
        case SUNSEARCH_SET_PITCH:
            setServoAngle(&pannel, sssm.pitch);
            sssm.state = SUNSEARCH_IDLE;
            break;
    }
}

void serializeTelemetry(Telemetry *t, uint8_t *b) {
    typedef union {
        uint16_t w;
        uint8_t n[2];
    } WORD;

    WORD dat;

    for (char i = 0; i < 3; i++) {
        dat.w = t->euler_raw[i];
        b[2*i] = dat.n[0];
        b[2*i + 1] = dat.n[1];
    }
    
    dat.w = t->light_sensor_raw;
    b[6] = dat.n[0];
    b[7] = dat.n[1];

    dat.w = t->solar_cell_raw;
    b[8] = dat.n[0];
    b[9] = dat.n[1];

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

    uint8_t buff[TX_BUFF_SZ] = {0};
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
            serializeTelemetry(&telem, buff);
            memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
            break;
        case TOGGLE_WPT:
            wpt_stat = !wpt_stat;
            enableChips(motor_stat, wpt_stat, imu_stat);
            buff[0] = wpt_stat;
            memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
            break;
        case SEARCH_SUN:
            if (srv->rx_buff[1] == 0x01) {
                buff[0] = sssm.state;
                memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                break;
            }
            if (wheels.running_1 || wheels.running_2) {
                buff[0] = 0x01;
                memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                break;
            }
            sssm.state = SUNSEARCH_SCAN_HEADING;
            buff[0] = 0x00;
            memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
            break;
        case MOVE:
            MoveMode mode = srv->rx_buff[1];
            enableChips((motor_stat = true), wpt_stat, imu_stat);
            switch (mode) {
                case STRAIGHT:
                    {
                    int8_t lev = (int8_t)(srv->rx_buff[2]);
                    uint16_t t = ((uint16_t)(srv->rx_buff[3]) << 8) | (srv->rx_buff[4]);
                    int res = driveWheelsForMs(&wheels, lev, lev, t, t);
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
                    int res = driveWheelsForMs(&wheels, lev1, lev2, t1, t2);
                    buff[0] = res;
                    memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                    break;
                    }
                case ABS_ANGLE:
                    {
                    uint16_t angle = ((uint16_t)(srv->rx_buff[2]) << 8) | (srv->rx_buff[3]);
                    int res = setTargetAngle(&wheels, angle);
                    buff[0] = res;
                    memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                    break;
                    }
                case DELTA_ANGLE:
                    {
                    int16_t angle = (int16_t)(((uint16_t)(srv->rx_buff[2]) << 8) | (srv->rx_buff[3]));
                    angle = wheels.pid->current + angle;
                    if (angle < 0) {
                        angle = 360 + angle;
                    }
                    int res = setTargetAngle(&wheels, angle);
                    buff[0] = res;
                    memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                    break;
                    }
            };
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
    gpio_set_dir_out_masked((1 << MOTOR_EN_PIN) | (1 << WPT_EN_PIN) | (1 << IMU_RST_PIN));
    gpio_pull_up(IMU_RST_PIN);
    enableChips(motor_stat, wpt_stat, (imu_stat = true));

    sleep_ms(3000);

    if (watchdog_caused_reboot()) {
        printf("Reseted by watchdog\r\n");
    }

    printf("Connecting to WiFi...\r\n");
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
        while (1) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(500);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(500);
        } // wait for watchdog reset
        return 1;
    }

    printf("Connected to Wifi! IP address is: %s\r\n", inet_ntoa(netif_default->ip_addr.addr));

    watchdog_enable(1000, true);

    printf("Setting up I2C IMU...\r\n");
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    watchdog_update();
    if (setupIMU(&imu)) {
        printf("Failed to init IMU, restarting...\r\n");
        watchdog_update();
        while (1) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(100);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(200);
        }
    }

    watchdog_update();

    printf("Setting up Analog Sensors...\r\n");
    adc_init();
    setupLightSensor();
    setupSolarMon();

    printf("Setting up Servo...\r\n");
    gpio_set_function(12, GPIO_FUNC_PWM);
    pannel.slice = pwm_gpio_to_slice_num(12);
    pannel.ch = PWM_CHAN_A;
    initServo(&pannel, 50, 500, 2500);
    gpio_set_function(13, GPIO_FUNC_PWM);
    parachute.slice = pwm_gpio_to_slice_num(13);
    parachute.ch = PWM_CHAN_B;
    initServo(&parachute, 100, 900, 2100);

    printf("Setting up Wheels...\r\n");
    gpio_set_function(8, GPIO_FUNC_PWM);
    gpio_set_function(9, GPIO_FUNC_PWM);
    gpio_set_function(10, GPIO_FUNC_PWM);
    gpio_set_function(11, GPIO_FUNC_PWM);
    wheels.slice_m1 = pwm_gpio_to_slice_num(8);
    wheels.slice_m2 = pwm_gpio_to_slice_num(10);
    initWheels(&wheels, &pid);

    printf("Setting up Server...\r\n");
    err_t e = server_init(&server, callback);

    if (e != ERR_OK) {
        printf("Failed to initialize server, restarting...\r\n");
        while (1) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(500);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(200);
        } // wait for watchdog reset
        return 1;
    }

    printf("Complete!\r\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    while (1) {
        watchdog_update();
        sleep_ms(10);
        if (!(wheels.pid->onTarget)) {
            uint16_t euler[3] = {0};
            (void)readIMU(&imu, euler);
            wheelsPIDTick(&wheels, euler[0]);
        }

        sunSearch();        
    }

    return 0;
}

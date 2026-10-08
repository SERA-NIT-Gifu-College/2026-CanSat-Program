#include "main.h"
#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/irq.h>
#include <hardware/pwm.h>
#include <hardware/regs/dreq.h>
#include <hardware/structs/xip_ctrl.h>
#include <hardware/sync.h>
#include <hardware/uart.h>
#include <hardware/watchdog.h>

#include <pico/cyw43_arch.h>
#include <pico/lwip_nosys.h>
#include <cyw43.h>
#include <cyw43_ll.h>
#include <lwip/inet.h>

#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/stdio_uart.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "imu.h"
#include "light_sensor.h"
#include "motor.h"
#include "server.h"
#include "solar.h"
#include "chip_enable.h"
#include "button.h"

bool motor_stat = false;
bool wpt_stat = false;
bool imu_stat = false;
bool hg_detected = false;
bool nm_detected = false;
bool lp2_configured = false;
bool skip_pre_launch = false;
bool readOutLT = false;
Server server = {0};
Telemetry telem = {0};
BNO055 imu = {
    .inst = i2c1,
    .addr = 0x28,
    .primary_mode = NDOF
};
Servo pannel = {0};
Servo parachute = {0};
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

    res = cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
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
        case SUNSEARCH_ROTATE_HEADING:
            break;
        case SUNSEARCH_SCAN_HEADING:
            {
            break;
            }
        case SUNSEARCH_ROTATE_BRIGHTEST:
            break;
        case SUNSEARCH_PITCH_UP:
            break;
        case SUNSEARCH_SCAN_PITCH:
            {
            break;
            }
        case SUNSEARCH_SET_PITCH:
            break;
        default:
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
        case LOAD_CALIB:
            loadCalibration(&imu);
            break;
        case SAVE_CALIB:
            saveCalibration(&imu);
            break;
        case TOGGLE_WPT:
            wpt_stat = !wpt_stat;
            enableChips(motor_stat, wpt_stat, imu_stat);
            buff[0] = wpt_stat;
            memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
            break;
        case TOGGLE_PARACHUTE:
            enableChips((motor_stat = true), wpt_stat, imu_stat);
            setServoAngle(&parachute, parachute.angle == 0.f ? 360.f : 0.f);
            sleep_ms(400);
            enableChips((motor_stat = false), wpt_stat, imu_stat);
            buff[0] = parachute.angle == 0.f ? 0x00 : 0x01;
            memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
            break;
        case SEARCH_SUN:
            if (srv->rx_buff[1] == 0x01) {
                buff[0] = sssm.state;
                memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                break;
            }
            if (sssm.state != SUNSEARCH_IDLE) {
                buff[0] = 0x01;
                memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                break;
            }
            sssm.state = SUNSEARCH_SCAN_HEADING;
            buff[0] = 0x00;
            memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
            break;
        case MOVE:
            {
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
                    int16_t ang = (int16_t)(((uint16_t)(srv->rx_buff[2]) << 8) | (srv->rx_buff[3]));
                    ang = wheels.target + ang;
                    if (ang < 0) {
                        ang = 360 + ang;
                    }
                    int res = setTargetAngle(&wheels, ang);
                    buff[0] = res;
                    memcpy(&(srv->tx_buff[0]), buff, TX_BUFF_SZ);
                    break;
                    }
                default:
                    break;
            };
            break;
            }
        case START_TELEM_EXTRACT:
            {
            readOutLT = true;
            break;
            }
        case PING:
            memcpy(&(srv->tx_buff[0]), "pong\r\n", 6);
            break;
        default:
            break;
    }
}

void imu_int_irq(uint _gpio, uint32_t _event_mask) {
    if (!hg_detected && !nm_detected) {
        hg_detected = true;
        printf("High G Detected!\r\n");
        return;
    }
    if (!nm_detected && hg_detected) {
        nm_detected = true;
        printf("No motion detected!\r\n");
        return;
    }
}

void init_select_irq() {
    if (!skip_pre_launch) {
        printf("Skipping pre launch program\r\n");
        skip_pre_launch = true;
        while (uart_is_readable(uart0)) {
            (void)uart_getc(uart0);
        } 
    }
}

int main(void) {
    stdio_uart_init_full(uart0, 9600, 0, 1);
    
    irq_set_exclusive_handler(UART0_IRQ, init_select_irq);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irqs_enabled(uart0, true, false);
    uart_set_fifo_enabled(uart0, false);
    printf("Hit any key to skip pre launch program\r\n");
    sleep_ms(5000);

    gpio_init_mask((1 << MOTOR_EN_PIN) | (1 << WPT_EN_PIN) | (1 << IMU_RST_PIN));
    gpio_set_dir_out_masked((1 << MOTOR_EN_PIN) | (1 << WPT_EN_PIN) | (1 << IMU_RST_PIN));
    gpio_pull_up(IMU_RST_PIN);
    enableChips(false, false, false);
    sleep_ms(500);
    enableChips(motor_stat, wpt_stat, (imu_stat = true));

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    sleep_ms(500);

    uart_set_irqs_enabled(uart0, false, false);
    irq_set_enabled(UART0_IRQ, false);
    uart_set_fifo_enabled(uart0, true);

    if (watchdog_caused_reboot()) {
        printf("Reseted by watchdog, continuing to post launch program.\r\n");
    } else if (!skip_pre_launch) {
        // PRELAUNCH / LAUNCH PROGRAM START
        printf("Starting pre launch/launch program\r\n");
        gpio_init(IMU_INT_PIN);
        gpio_set_dir(IMU_INT_PIN, false);
        gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, imu_int_irq);
        // Enough for ~7 minutes of acceleration data with 100 samples per second
        uint8_t telemBuff[LAUNCH_TIME_TELEM_SZ] = {0}; 
        uint32_t i = 0;
        if (setupIMULaunchPhase(&imu, 0)) {
            printf("Failed to setup IMU for high G detection. Please manually restart the device.\r\n");
            while (1) ;
        }
        while (!getBootselBtn()) ;
        printf("Calibrating for acceleration offset, let the rover still in launch position...\r\n");
        int16_t o[3] = {0};
        for (int x = 0; x < 10; x++) {
            int16_t b[3] = {0};
            getAcc(&imu, (uint8_t*)(&b[0]));
            o[0] += b[0];
            o[1] += b[1];
            o[2] += b[2];
            sleep_ms(1000);
        }
        o[0] = o[0] / 10;
        o[1] = o[1] / 10;
        o[2] = o[2] / 10;
        printf("Complete!\r\n");
        sleep_ms(100);
        uint8_t buff[6] = {0};
        int16_t *t = (int16_t*)(&buff[0]);
        while (1) {
            /*
            if (hg_detected && !lp2_configured) {
                clearINT(&imu);
                if (setupIMULaunchPhase(&imu, 1)) {
                    printf("Failed to setup IMU for no motion detection. Please manually restart the device.\r\n");
                    while (1) ;
                }
                lp2_configured = true;
            }
            if (hg_detected && nm_detected) {
                printf("Detected Landing, rebooting into post launch program...\r\n");
                int ret = writeLaunchTelem(telemBuff);
                if (ret) printf("Failed to save telemetry data\r\n");
                watchdog_enable(5000, true);
                while (1) ;
            }
            if (i+6 < 256 * 1024) {
                getAcc(&imu, buff);
                t[0] -= o[0];
                t[1] -= o[1];
                t[2] -= o[2];
                for (int j = 0; j < 6; j++) {
                    telemBuff[i+j] = buff[j];
                }
                i += 6;
            }
            sleep_ms(10);
            */
            getAcc(&imu, buff);
            printf("%lf, %lf, %lf\r\n",
                (double)t[0] / 100.0,
                (double)t[1] / 100.0,
                (double)t[2] / 100.0
            );
            sleep_ms(200);
        }
        // PRELAUNCH / LAUNCH PROGRAM END
    }

    // POST LAUNCH PROGRAM BELOW //

    printf("Starting post launch program...\r\n");

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

    adc_init();
    setupLightSensor();
    setupSolarMon();

    enableChips((motor_stat = true), wpt_stat, imu_stat);
    gpio_set_function(12, GPIO_FUNC_PWM);
    pannel.slice = pwm_gpio_to_slice_num(12);
    pannel.ch = PWM_CHAN_A;
    initServo(&pannel, 50, 500, 2500);
    gpio_set_function(13, GPIO_FUNC_PWM);
    parachute.slice = pwm_gpio_to_slice_num(13);
    parachute.ch = PWM_CHAN_B;
    initServo(&parachute, 100, 900, 2700);

    gpio_set_function(8, GPIO_FUNC_PWM);
    gpio_set_function(9, GPIO_FUNC_PWM);
    gpio_set_function(10, GPIO_FUNC_PWM);
    gpio_set_function(11, GPIO_FUNC_PWM);
    wheels.slice_m1 = pwm_gpio_to_slice_num(8);
    wheels.slice_m2 = pwm_gpio_to_slice_num(10);
    initWheels(&wheels);
    enableChips((motor_stat = false), wpt_stat, imu_stat);

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
        if (readOutLT) {
            const uint8_t *t = getLaunchTelemAddr();
            for (uint32_t i = 0; i < 256 * 1024; i++) {
                uart_putc(uart0, t[i]);
                if (i % 512 == 0) watchdog_update();
            }
            readOutLT = false;
        }
        int s = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
        switch (s) {
            case CYW43_LINK_FAIL:
                printf("Connection failed\r\n");
                break;
            case CYW43_LINK_NOIP:
                printf("No IP assigned\r\n");
                break;
            case CYW43_LINK_DOWN:
                printf("Connection is down\r\n");
                break;
            case CYW43_LINK_NONET:
                printf("No matching SSID\r\n");
                break;
            default:
                break;
        }
        if (!(wheels.running_1 || wheels.running_2)) {
            enableChips((motor_stat = false), wpt_stat, imu_stat);
        }

        sunSearch();        
    }

    return 0;
}

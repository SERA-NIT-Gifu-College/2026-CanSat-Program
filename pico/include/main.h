#ifndef __MAIN_H__
#define __MAIN_H__

/**
 * @file main.h
 * @brief Major functionalities
 */

/**
 * Connects Wifi, sets watchdog timer if failed
 * @return error code, 0 for success, 1 for initialization error, 2 for connection timeout
 */
int connectWifi(void);

/**
 * Put pico into deep sleep with specified time and chip enable state
 * @param ms Time period in millisecond to put in deep sleep
 * @param ce Chip enable state @see Chips
 * @return 0 in success, 1 in failiure
 */
int go_sleep(uint32_t ms, uint8_t ce);

/**
 * A callback for server, executes different set of commands depending on recived command
 */
void callback(void *arg);

/**
 * Main entry point of the firmware
 * @return pseudo error code, 0 for success, 1 for error
 */
int main(void);

#endif // __MAIN_H__

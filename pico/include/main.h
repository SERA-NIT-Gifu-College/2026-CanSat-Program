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
 * Main entry point of the firmware
 * @return pseudo error code, 0 for success, 1 for error
 */
int main(void);

#endif // __MAIN_H__

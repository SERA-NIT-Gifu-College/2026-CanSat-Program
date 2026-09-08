#ifndef __MAIN_H__
#define __MAIN_H__

#include "server.h"

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
 * Serializes telemetry structure into byte stream
 * @param t A pointer to telemetry structure
 * @param b A pointer to byte-sized buffer to be written
 */
void serializeTelemetry(Telemetry *t, uint8_t *b);

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

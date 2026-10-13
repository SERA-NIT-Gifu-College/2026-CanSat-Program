#ifndef __APP_HPP__
#define __APP_HPP__

/**
 * @file app.hpp
 * @brief 2026 CanSat client/controller program
 */

#include "tcp_client.hpp"

/**
 * Copy of MoveMode in firmware
 */
typedef enum {
    STRAIGHT,
    INDEPENDENT,
    ABS_ANGLE,
    DELTA_ANGLE,
    STOP,
    NUM_MOVE_MODE
} MoveMode;

/**
 * Parameters for move commands
 */
typedef struct {
    int8_t speed_1;
    int8_t speed_2;
    int16_t angle;
    uint16_t time_1;
    uint16_t time_2;
} MoveParam;

/**
 * Ping operation
 * @param count number of pings to send
 * @param interval interval in milliseconds between pings
 * @param c a reference to a TCP client object
 * @return status code, 0 for success, 1 for error
 */
auto ping(int count, int interval, TCP_Client &c) -> int;
/**
 * Telemetry retrieval operation
 * @param interval interval in milliseconds between command
 * @param c a reference to a TCP client object
 * @return status code, 0 for success, 1 for error
 */
auto get_telemetry(int interval, TCP_Client &c) -> int;
/**
 * Move operation
 * @param m Move mode to be performed
 * @param p a pointer to move command parameters structure
 * @param c a reference to a TCP client object
 * @return status code, 0 for success, 1 for error
 */
auto move_rover(MoveMode m, MoveParam *p, TCP_Client &c) -> int;

/**
 * Main entry point of this program
 * @param argc number of command line arguments
 * @param argv array of arguments
 * @return status code, 0 for success, 1 for error
 */
auto main(int argc, char **argv) -> int;

#endif // __APP_HPP__

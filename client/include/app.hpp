#ifndef __APP_HPP__
#define __APP_HPP__

#include "tcp_client.hpp"

typedef enum {
    STRAIGHT,
    INDEPENDENT,
    ABS_ANGLE,
    DELTA_ANGLE,
    STOP,
    NUM_MOVE_MODE
} MoveMode;

typedef struct {
    int8_t speed_1;
    int8_t speed_2;
    int16_t angle;
    uint16_t time_1;
    uint16_t time_2;
} MoveParam;

auto ping(int count, int interval, TCP_Client &c) -> int;
auto get_telemetry(int interval, TCP_Client &c) -> int;
auto move_rover(MoveMode m, MoveParam *p, TCP_Client &c) -> int;

auto main(int argc, char **argv) -> int;

#endif // __APP_HPP__

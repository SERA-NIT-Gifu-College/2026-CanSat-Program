#ifndef __APP_HPP__
#define __APP_HPP__

#include "tcp_client.hpp"

auto ping(int count, int interval, TCP_Client &c) -> int;
auto get_telemetry(TCP_Client &c) -> int;

auto main(int argc, char **argv) -> int;

#endif // __APP_HPP__

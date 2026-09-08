#include "app.hpp"
#include "tcp_client.hpp"

#include <iostream>
#include <string>
#include <cstdint>
#include <cstdio>
#include <boost/asio.hpp>
#include <cxxopts.hpp>

boost::asio::io_context io_ctx;

auto ping(int count, int interval, TCP_Client &c) -> int {
    for (int i = 0; i < count; i++) {
        std::cout << "<< ping" << std::endl;
        auto res = c.sendCommand(TCP_Client::PING, 0);
        if (res.has_value()) {
            std::cout << ">> ";
            auto a = res.value();

            for (auto& i : a) {
                std::cout << i;
                if (i == '\0')
                    break;
            }
            std::cout << std::endl;
        } else {
            std::cerr << "No result" << std::endl;
            return 1;
        }

        if (i < (count-1)) {
            boost::asio::steady_timer t(io_ctx, boost::asio::chrono::seconds(interval));
            t.wait();
        }
    }

    return 0;
}

auto get_telemetry(TCP_Client &c) -> int {
    auto res = c.sendCommand(TCP_Client::GET_TELEMETRY, 0);
    if (!(res.has_value())) {
        std::cerr << "Error on obtaining data" << std::endl;
        return 1;
    }
    auto data = res.value();

    int16_t euler_heading =
        (int16_t)((uint16_t)(data[1] << 8) | data[0]) / 16;
    int16_t euler_roll =
        (int16_t)((uint16_t)(data[3] << 8) | data[2]) / 16;
    int16_t euler_pitch =
        (int16_t)((uint16_t)(data[5] << 8) | data[4]) / 16;
    double light_sensor_voltage =
        ((uint16_t)(data[7] << 8) | data[6]) * 3.3 / (1 << 12);
    double solar_cell_voltage =
        ((uint16_t)(data[9] << 8) | data[8]) * 3.3 / (1 << 12);

    std::cout << "Euler: (" << euler_heading << ", " << euler_roll << ", " << euler_pitch << ")" << std::endl;

    std::cout << "Light Sensor: " << light_sensor_voltage << " [V]" << std::endl;
    std::cout << "Solar Cell: " << solar_cell_voltage << " [V]" << std::endl;

    std::printf("IMU Status: 0x%02X\n", data[10]);
    std::printf("IMU Error: 0x%02X\n", data[11]);
    std::printf("IMU Calib: 0x%02X\n", data[12]);
    
    return 0;
}

auto main(int argc, char **argv) -> int {
    cxxopts::Options options("cansatClient", "Controller client for 2026 CanSat rover");

    options.add_options()
        ("c,ping-count", "Number of ping to send", cxxopts::value<int>()->default_value("1"))
        ("i,info", "Read out telemetry information")
        ("p,ping", "Send PING packet to rover")
        ("t,period", "Ping interval", cxxopts::value<int>())
        ("h,help", "Print usage")
        ("ip", "IP address of rover's server", cxxopts::value<std::string>())
        ("port", "Port number of rover's server", cxxopts::value<int>())
        ;

    options.parse_positional({"ip", "port"});

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    if (result.count("ip") == 0 || result.count("port") == 0) {
        std::cerr << "No IP or port is specified" << std::endl;
        return 1;
    }

    std::string ip_addr = result["ip"].as<std::string>();
    int port = result["port"].as<int>();

    TCP_Client client(io_ctx, ip_addr, port);

    if (result["info"].as<bool>()) {
        int res = get_telemetry(client);
        if (res != 0) {
            std::cerr << "Failed to retrive telemetry" << std::endl;
            return 1;
        }
    } else if (result["ping"].as<bool>() && result.count("period")) {
        int count = result["ping-count"].as<int>();
        int interval = result["period"].as<int>();
        if (interval < 0) {
            std::cerr << "Interval should be non-zero positive integer" << std::endl;
            return 1;
        }
        ping(count, interval, client);
    } 
    return 0;
}

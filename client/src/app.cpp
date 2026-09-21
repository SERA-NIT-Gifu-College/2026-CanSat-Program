#include "app.hpp"
#include "tcp_client.hpp"

#include <array>
#include <iostream>
#include <map>
#include <string>
#include <cstdint>
#include <cstdio>
#include <boost/asio.hpp>
#include <cxxopts.hpp>

boost::asio::io_context io_ctx;

auto ping(int count, int interval, TCP_Client &c) -> int {
    for (int i = 0; i < count; i++) {
        std::cout << "<< ping" << std::endl;
        std::array<uint8_t, 7> opt = {0};
        auto res = c.sendCommand(TCP_Client::PING, opt);
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
            boost::asio::steady_timer t(io_ctx, boost::asio::chrono::milliseconds(interval));
            t.wait();
        }
    }

    return 0;
}

auto get_telemetry(int interval, TCP_Client &c) -> int {
    std::array<uint8_t, 7> opt = {0};
    auto res = c.sendCommand(TCP_Client::GET_TELEMETRY, opt);
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

    if (interval != 0) {
        boost::asio::steady_timer t(io_ctx, boost::asio::chrono::milliseconds(interval));
        t.wait();
        std::printf("\x1b[2J\x1b[H");
    }
    
    return 0;
}

auto move_rover(MoveMode m, MoveParam *p, TCP_Client &c) -> int {
    std::array<uint8_t, 7> opt = {0};
    opt[0] = (uint8_t)m;

    switch (m) {
        case MoveMode::STRAIGHT:
            opt[1] = (uint8_t)(p->speed_1);
            opt[2] = (uint8_t)((p->time_1 & 0xFF00) >> 8);
            opt[3] = (uint8_t)(p->time_1 & 0x00FF);
            break;
        case MoveMode::INDEPENDENT:
            opt[1] = (uint8_t)(p->speed_1);
            opt[2] = (uint8_t)((p->time_1 & 0xFF00) >> 8);
            opt[3] = (uint8_t)(p->time_1 & 0x00FF);
            opt[4] = (uint8_t)(p->speed_2);
            opt[5] = (uint8_t)((p->time_2 & 0xFF00) >> 8);
            opt[6] = (uint8_t)(p->time_2 & 0x00FF);
            break;
        case MoveMode::ABS_ANGLE:
            if (p->angle < 0 || p->angle > 360) {
                std::cout << "Angle out of range" << std::endl;
                return 1;
            }
        case MoveMode::DELTA_ANGLE:
            opt[1] = (uint8_t)((p->angle & 0xFF00) >> 8);
            opt[2] = (uint8_t)(p->angle & 0x00FF);
            break;
        default:
            std::cout << "Invalid Mode" << std::endl;
            return 1;
    };

    auto res = c.sendCommand(TCP_Client::MOVE, opt);
    if (!(res.has_value())) {
        std::cerr << "Error on obtaining data" << std::endl;
        return 1;
    }
    auto data = res.value();

    std::cout << (data[0] == 0 ? "Accepted" : "Rejected") << std::endl;

    return 0;
}

auto main(int argc, char **argv) -> int {
    cxxopts::Options options("cansatClient", "Controller client for 2026 CanSat rover");

    options.add_options()
        ("i,info", "Read out telemetry information")
        ("f,follow", "Follow/Poll telemetry")
        ("p,ping", "Send PING packet to rover")
        ("c,ping-count", "Number of ping to send", cxxopts::value<int>()->default_value("1"))
        ("m,move", "Drive mode of rover, options are: straight, independent, abs_angle, delta_angle", cxxopts::value<std::string>())
        ("s,speed", "PWM value(s) for rover drive, value(s) is 0~100", cxxopts::value<std::vector<int>>())
        ("a,angle", "Target angle for abs_angle or angle difference for delta_angle", cxxopts::value<int>())
        ("t,timeout", "Ping interval, telemetry poll interval or move command duration in milliseconds; use list of two intervals for independent move command", cxxopts::value<std::vector<int>>()->default_value("1000,0"))
        ("w,wireless-power-transmit", "Toggles wireless power transmittion")
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
        int interval = 0;
        if (result["follow"].as<bool>()) {
            interval = result["timeout"].as<std::vector<int>>()[0];
            if (interval < 0) {
                std::cerr << "Interval should be non-zero positive integer" << std::endl;
                return 1;
            }
        } 

        do {
            int res = get_telemetry(interval, client);
            if (res != 0) {
                std::cerr << "Failed to retrive telemetry" << std::endl;
                return 1;
            }
        } while (result["follow"].as<bool>());
    } else if (result["ping"].as<bool>()) {
        int count = result["ping-count"].as<int>();
        int interval = result["timeout"].as<std::vector<int>>()[0];
        if (interval < 0) {
            std::cerr << "Interval should be non-zero positive integer" << std::endl;
            return 1;
        }
        int res = ping(count, interval, client);
        if (res) {
            std::cerr << "Failed to send PING" << std::endl;
            return 1;
        }
    } else if (result["wireless-power-transmit"].as<bool>()) {
        std::array<uint8_t, 7> opt = {0};
        auto res = client.sendCommand(TCP_Client::TOGGLE_WPT, opt);
        if (!(res.has_value())) {
            std::cerr << "Failed to toggle WPT state" << std::endl;
            return 1;
        }
        auto data = res.value();
        std::cout << (data[0] == 0x01 ? "WPT ON" : "WPT OFF") << std::endl;
    } else if (result.count("move")) {
        const std::map<std::string, MoveMode> modes{
            {"straight", MoveMode::STRAIGHT},
            {"independent", MoveMode::INDEPENDENT},
            {"abs_angle", MoveMode::ABS_ANGLE},
            {"delta_angle", MoveMode::DELTA_ANGLE}
        };

        auto m = result["move"].as<std::string>();

        if (!modes.contains(m)) {
            std::cerr << "Invalid mode" << std::endl;
            return 1;
        }

        MoveMode mode = modes.at(m);
        MoveParam param = {0};

        switch (mode) {
            case MoveMode::STRAIGHT:
                param.speed_1 = result["speed"].as<std::vector<int>>()[0];
                param.time_1 = (uint16_t)(result["timeout"].as<std::vector<int>>()[0]);
                break;
            case MoveMode::INDEPENDENT:
                param.speed_1 = result["speed"].as<std::vector<int>>()[0];
                param.time_1 = (uint16_t)(result["timeout"].as<std::vector<int>>()[0]);
                param.speed_2 = result["speed"].as<std::vector<int>>()[1];
                param.time_2 = (uint16_t)(result["timeout"].as<std::vector<int>>()[1]);
                break;
            case MoveMode::ABS_ANGLE:
            case MoveMode::DELTA_ANGLE:
                param.angle = result["angle"].as<int>();
            default:
                break;
        }

        int res = move_rover(mode, &param, client);

        if (res) {
            std::cerr << "Failed to send command" << std::endl;
            return 1;
        }
    }

    return 0;
}

#include "app.hpp"
#include "tcp_client.hpp"

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <cxxopts.hpp>

boost::asio::io_context io_ctx;

auto ping(int count, TCP_Client &c) -> int {
    for (int i = 0; i < count; i++) {
        std::cout << "<< ping" << std::endl;
        auto res = c.sendCommand(TCP_Client::PING);
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

        boost::asio::steady_timer t(io_ctx, boost::asio::chrono::seconds(1));
        t.wait();
    }

    return 0;
}

auto main(int argc, char **argv) -> int {
    cxxopts::Options options("cansatClient", "Controller client for 2026 CanSat rover");

    options.add_options()
        ("p,ping", "Send PING packet to rover")
        ("c,ping-count", "Number of ping to send", cxxopts::value<int>()->default_value("1"))
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

    std::cout << ip_addr << ":" << port << std::endl;

    TCP_Client client(io_ctx, ip_addr, port);

    int count = result["ping-count"].as<int>();
    ping(count, client);

    return 0;
}

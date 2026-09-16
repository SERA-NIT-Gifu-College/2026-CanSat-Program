#include "tcp_client.hpp"
#include <iostream>

using boost::asio::ip::tcp;
using boost::asio::io_context;

TCP_Client::TCP_Client(io_context &ctx, std::string &addr_str, unsigned int port) :
    sckt_(ctx),
    server_ep_(boost::asio::ip::make_address(addr_str), port)
{}

std::optional<std::array<uint8_t, 16>> TCP_Client::sendCommand(TCP_Client::Command cmd, std::array<uint8_t, 7> &option) {
    cmd_ = cmd;
    option_ = option;
    int res = 0;
    res += do_tx();
    res += do_rx() << 1;
    
    if (res) {
        std::cerr << "Could not send command: " << res << std::endl;
        return std::nullopt;
    }

    sckt_.close();

    return rx_buff_;
}

int TCP_Client::do_rx() {
    try {
        boost::system::error_code ec;
        size_t len = sckt_.read_some(boost::asio::buffer(rx_buff_), ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}

int TCP_Client::do_tx() {
    try {
        sckt_.connect(server_ep_);
        tx_buff_[0] = (uint8_t)cmd_;
        for (int i = 0; i < 7; i++) {
            tx_buff_[1 + i] = option_[i];
        }
        sckt_.write_some(boost::asio::buffer(tx_buff_));
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}

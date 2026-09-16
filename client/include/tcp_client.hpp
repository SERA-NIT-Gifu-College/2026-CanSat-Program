#ifndef __TCP_CLIENT_HPP__
#define __TCP_CLIENT_HPP__

#include <array>
#include <atomic>
#include <string>
#include <optional>
#include <mutex>
#include <boost/asio.hpp>

class TCP_Client {
    public:
        typedef enum {
            GET_TELEMETRY,
            LOAD_CALIB,
            SAVE_CALIB,
            SEARCH_SUN,
            MOVE,
            TOGGLE_WPT,
            PING,
            NUM_COMMANDS
        } Command;

        TCP_Client(boost::asio::io_context &ctx, std::string &addr_str, unsigned int port);

        std::optional<std::array<uint8_t, 16>> sendCommand(Command cmd, std::array<uint8_t, 7> &option);
    private:
        int do_rx();
        int do_tx();

        boost::asio::ip::tcp::socket sckt_;
        boost::asio::ip::tcp::endpoint server_ep_;

        Command cmd_;
        std::array<uint8_t, 7> option_;

        std::array<uint8_t, 8> tx_buff_;
        std::array<uint8_t, 16> rx_buff_;
        std::mutex rx_buff_mutex_;
};

#endif // __TCP_CLIENT_HPP__

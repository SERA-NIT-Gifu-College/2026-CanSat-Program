#ifndef __TCP_CLIENT_HPP__
#define __TCP_CLIENT_HPP__

/**
 * @file tcp_client.hpp
 * @brief TCP Client codes
 */

#include <array>
#include <string>
#include <optional>
#include <boost/asio.hpp>

class TCP_Client {
    public:
        /**
         * Copy of server commands from firmware
         */
        typedef enum {
            GET_TELEMETRY,
            LOAD_CALIB,
            SAVE_CALIB,
            SEARCH_SUN,
            MOVE,
            TOGGLE_WPT,
            TOGGLE_PARACHUTE,
            START_TELEM_EXTRACT,
            WD_RESET,
            PING,
            NUM_COMMANDS
        } Command;

        /**
         * Constructor
         * @param ctx a reference to ASIO IO context
         * @param addr_str an address to CanSat as IPv4 string
         * @param port a port number to CanSat server
         */
        TCP_Client(boost::asio::io_context &ctx, std::string &addr_str, unsigned int port);

        /**
         * Sends command to CanSat
         * @param cmd a command to be issued
         * @param option 7 bytes including options for some commands
         * @return an optional value wraps 16 bytes response
         */
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
};

#endif // __TCP_CLIENT_HPP__

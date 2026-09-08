#ifndef __SERVER_H__
#define __SERVER_H__

/**
 * @file server.h
 * @brief Functions for server operation
 */

#include <pico/cyw43_arch.h>
#include <lwip/pbuf.h>
#include <lwip/tcp.h>

#define TCP_PORT 8000 //!< TCP port on which server runs
#define POLL_TIME 5 //!< tcp_poll interval time in seconds
#define TX_BUFF_SZ 16 //!< Transmission buffer size in bytes
#define RX_BUFF_SZ 8 //!< Recive buffer size in bytes

#define PING_PACKET_TX_LEN 7                //!< TX packet size for PING
#define GET_TELEMETRY_PACKET_TX_LEN 16      //!< TX packet size for READ_IMU
#define LOAD_CALIB_PACKET_TX_LEN 1          //!< TX packet size for LOAD_CALIB
#define SAVE_CALIB_PACKET_TX_LEN 1          //!< TX packet size for SAVE_CALIB
#define SEARCH_SUN_PACKET_TX_LEN 1          //!< TX packet size for SEARCH_SUN

/** Function prototype for callback function */
typedef void (*Data_Preparation_Callback)(void*);

/** Available commands for this server */
typedef enum {
    GET_TELEMETRY,
    LOAD_CALIB,
    SAVE_CALIB,
    SEARCH_SUN,
    PING,
    NUM_COMMAND
} Command;

/** Telemetry packet definition */
typedef struct {
    uint16_t euler_raw[3];
    uint16_t light_sensor_raw;
    uint16_t solar_cell_raw;
    uint8_t imu_status;
    uint8_t imu_error;
    uint8_t imu_calib;
} Telemetry;

/** Server descriptor */
typedef struct {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    uint8_t tx_buff[TX_BUFF_SZ];
    uint8_t rx_buff[RX_BUFF_SZ];
    uint8_t tx_len;
    uint8_t rx_len;
    uint8_t status;
    uint8_t poll_count;
    Command cmd;
    Data_Preparation_Callback cb;
} Server;

/**
 * Initializes server
 * @param srv a pointer to a server descriptor
 * @param cb callback function, called in server_rx @see Data_Preparation_Callback
 * @return lwip error code
 */
err_t server_init(Server *srv, Data_Preparation_Callback cb);
/**
 * Closes server and connection between client
 * @param arg a pointer to a server descriptor
 * @return lwip error code
 */
err_t server_close(void *arg);
/**
 * Closes connection between client
 * @param arg a pointer to a server descriptor
 * @return lwip error code
 */
static err_t server_client_disconn(void *arg);
/**
 * Functions that defines how to transmit data to client
 * @param arg a pointer to a server descriptor
 * @param conn_pcb connected client
 * @return lwip error code
 */
static err_t server_data_tx(void *arg, struct tcp_pcb *conn_pcb);

/** see lwip documentation */
static err_t server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
/** see lwip documentation */
static err_t server_poll(void *arg, struct tcp_pcb *conn_pcb);
/** see lwip documentation */
static void server_err(void *arg, err_t err);
/** see lwip documentation */
static err_t server_tx(void *arg, struct tcp_pcb *conn_pcb, uint16_t len);
/** see lwip documentation */
static err_t server_rx(void *arg, struct tcp_pcb *conn_pcb, struct pbuf *rx_data, err_t err);

#endif // __SERVER_H__

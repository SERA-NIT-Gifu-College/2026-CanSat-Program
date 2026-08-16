#ifndef __SERVER_H__
#define __SERVER_H__

#include <pico/cyw43_arch.h>
#include <lwip/pbuf.h>
#include <lwip/tcp.h>

#define TCP_PORT 8000
#define POLL_TIME 5
#define TX_BUFF_SZ 16
#define RX_BUFF_SZ 8

typedef enum {
    READ_IMU,
    LOAD_CALIB,
    SAVE_CALIB,
    SEARCH_SUN,
    START_SELF_CHARGE,
    START_POWER_TRANS,
    STOP_GENERATOR,
    PING,
} Command;

typedef struct {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    uint8_t tx_buff[TX_BUFF_SZ];
    uint8_t rx_buff[RX_BUFF_SZ];
    uint8_t tx_len;
    uint8_t rx_len;
    uint8_t status;
    Command cmd;
    bool newCommandIssued;
} Server;

err_t server_init(Server *srv);
static err_t server_close(void *arg);
static err_t server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t server_poll(void *arg, struct tcp_pcb *conn_pcb);
static void server_err(void *arg, err_t err);
static err_t server_data_tx(void *arg, struct tcp_pcb *conn_pcb);
static err_t server_tx(void *arg, struct tcp_pcb *conn_pcb, uint16_t len);
static err_t server_rx(void *arg, struct tcp_pcb *conn_pcb, struct pbuf *rx_data, err_t err);
static err_t server_client_disconn(void *arg);

#endif // __SERVER_H__

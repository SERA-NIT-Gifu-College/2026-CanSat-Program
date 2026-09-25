#include "server.h"
#include <lwip/err.h>
#include <pico/cyw43_arch.h>
#include <lwip/pbuf.h>
#include <lwip/tcp.h>
#include <string.h>
#include <stdio.h>

static uint8_t cmd_tx_lens[NUM_COMMAND] = {
    GET_TELEMETRY_PACKET_TX_LEN,
    LOAD_CALIB_PACKET_TX_LEN,
    SAVE_CALIB_PACKET_TX_LEN,
    SEARCH_SUN_PACKET_TX_LEN,
    MOVE_PACKET_TX_LEN,
    TOGGLE_WPT_PACKET_TX_LEN,
    TOGGLE_PARACHUTE_PACKET_TX_LEN,
    PING_PACKET_TX_LEN
};

err_t server_init(Server *srv, Data_Preparation_Callback cb) {
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);

    if (!pcb) {
        return ERR_VAL;
    }

    err_t e = tcp_bind(pcb, NULL, TCP_PORT);
    if (e) {
        return e;
    }

    srv->server_pcb = tcp_listen_with_backlog(pcb, 1);

    if (!(srv->server_pcb)) {
        if (pcb) tcp_close(pcb);

        return ERR_ABRT;
    }

    tcp_arg(srv->server_pcb, (void*)srv);
    tcp_accept(srv->server_pcb, server_accept);

    srv->cb = cb;
    srv->poll_count = 0;

    return ERR_OK;
}

static err_t server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    Server *srv = (Server*)arg;

    if (err != ERR_OK || newpcb == NULL) {
        return ERR_VAL;
    }

    srv->client_pcb = newpcb;
    tcp_arg(newpcb, srv);
    tcp_sent(newpcb, server_tx);
    tcp_recv(newpcb, server_rx);
    tcp_err(newpcb, server_err);
    tcp_poll(newpcb, server_poll, POLL_TIME * 2);

    return ERR_OK;
}

static err_t server_client_disconn(void *arg) {
    Server *srv = (Server*)arg;

    err_t err = ERR_OK;

    if (srv->client_pcb != NULL) {
        tcp_arg(srv->client_pcb, NULL);
        tcp_sent(srv->client_pcb, NULL);
        tcp_recv(srv->client_pcb, NULL);
        tcp_err(srv->client_pcb, NULL);
        tcp_poll(srv->client_pcb, NULL, 0);
        err = tcp_close(srv->client_pcb);
        if (err != ERR_OK) {
            tcp_abort(srv->client_pcb);
            err = ERR_ABRT;
        }
        srv->client_pcb = NULL;
    }

    return err;
}

err_t server_close(void *arg) {
    Server *srv = (Server*)arg;

    err_t err = ERR_OK;

    err = server_client_disconn(arg);

    if (srv->server_pcb) {
        tcp_arg(srv->server_pcb, NULL);
        tcp_close(srv->server_pcb);
        srv->server_pcb = NULL;
    }

    return err;
}

static err_t server_poll(void *arg, struct tcp_pcb *conn_pcb) {
    Server *srv = (Server*)arg;

    srv->poll_count++;

    if (srv->poll_count > 3) {
        printf("timeout detected disconnecting...\r\n");
        srv->poll_count = 0;
        return server_client_disconn(arg);
    }

    return ERR_OK;
}

static void server_err(void *arg, err_t err) {
    printf("server_err: %d\r\n", err);
    if (err != ERR_ABRT) {
        printf("Critical Error, restarting server\r\n");
        server_close(arg);
        server_init((Server*)arg, ((Server*)arg)->cb);
    }
}

static err_t server_data_tx(void *arg, struct tcp_pcb *conn_pcb) {
    Server *srv = (Server*)arg;
    
    uint8_t len = 0;

    srv->tx_len = 0;

    len = cmd_tx_lens[srv->cmd];

    cyw43_arch_lwip_check();

    err_t err = tcp_write(conn_pcb, srv->tx_buff, len <= TX_BUFF_SZ ? len : TX_BUFF_SZ, TCP_WRITE_FLAG_COPY);

    if (err != ERR_OK) {
        printf("Error on sending\r\n");
        return server_client_disconn(arg);
    }

    return ERR_OK;
}

static err_t server_tx(void *arg, struct tcp_pcb *conn_pcb, uint16_t len) {
    Server *srv = (Server*)arg;

    srv->tx_len += len;

    return ERR_OK;
}

static err_t server_rx(void *arg, struct tcp_pcb *conn_pcb, struct pbuf *rx_data, err_t err) {
    Server *srv = (Server*)arg;

    if (!rx_data) {
        printf("!rx_data\r\n");
        (void)server_client_disconn(arg);
        return ERR_VAL;
    }

    cyw43_arch_lwip_check();

    if (rx_data->tot_len > 0) {
        uint8_t *p = (uint8_t*)(rx_data->payload);
        memcpy(srv->rx_buff, p, RX_BUFF_SZ);
        pbuf_free(rx_data);
        srv->cmd = srv->rx_buff[0];

        (*(srv->cb))(arg);

        return server_data_tx(arg, conn_pcb);
    }

    pbuf_free(rx_data);

    return ERR_OK;
}

#include "server.h"
#include <lwip/err.h>
#include <pico/cyw43_arch.h>
#include <lwip/tcp.h>
#include <string.h>
#include <stdio.h>

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

    return ERR_OK;
}

static err_t server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    printf("Accept\r\n");
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
    printf("client_disconn\r\n");
    Server *srv = (Server*)arg;

    err_t err = ERR_OK;

    if (srv->client_pcb != NULL) {
        tcp_arg(srv->client_pcb, NULL);
        tcp_sent(srv->client_pcb, server_tx);
        tcp_recv(srv->client_pcb, server_rx);
        tcp_err(srv->client_pcb, server_err);
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

static err_t server_close(void *arg) {
    printf("Close\r\n");
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
    printf("Poll\r\n");

    return ERR_OK;
}

static void server_err(void *arg, err_t err) {
    printf("Err\r\n");
    if (err != ERR_ABRT) {
        server_close(arg);
    }
}

static err_t server_data_tx(void *arg, struct tcp_pcb *conn_pcb) {
    printf("data_tx\r\n");
    Server *srv = (Server*)arg;
    
    uint8_t len = 0;

    switch (srv->cmd) {
        case READ_IMU:
            len = READ_IMU_PACKET_TX_LEN;
            break;
        case GET_IMU_STATUS:
            len = GET_IMU_STATUS_PACKET_TX_LEN;
            break;
        case SAVE_CALIB:
            len = SAVE_CALIB_PACKET_TX_LEN;
            break;
        case LOAD_CALIB:
            len = LOAD_CALIB_PACKET_TX_LEN;
            break;
        case START_SELF_CHARGE:
            len = START_SELF_CHARGE_PACKET_TX_LEN;
            break;
        case START_POWER_TRANS:
            len = START_POWER_TRANS_PACKET_TX_LEN;
            break;
        case SEARCH_SUN:
            len = SEARCH_SUN_PACKET_TX_LEN;
            break;
        case STOP_GENERATOR:
            len = STOP_GENERATOR_PACKET_TX_LEN;
            break;
        case PING:
            len = PING_PACKET_TX_LEN;
            break;
        case NUM_COMMAND:
            return ERR_VAL;
        default:
            len = TX_BUFF_SZ;
    }

    srv->tx_len = 0;

    cyw43_arch_lwip_check();

    err_t err = tcp_write(conn_pcb, srv->tx_buff, len <= TX_BUFF_SZ ? len : TX_BUFF_SZ, TCP_WRITE_FLAG_COPY);

    if (err != ERR_OK) {
        printf("Failed tx: %d\r\n", err);
        return server_close(arg);
    }

    return ERR_OK;
}

static err_t server_tx(void *arg, struct tcp_pcb *conn_pcb, uint16_t len) {
    printf("tx\r\n");
    Server *srv = (Server*)arg;

    srv->tx_len += len;

    return server_client_disconn(arg);
}

static err_t server_rx(void *arg, struct tcp_pcb *conn_pcb, struct pbuf *rx_data, err_t err) {
    printf("rx\r\n");
    Server *srv = (Server*)arg;

    if (!rx_data) {
        printf("rx ERR_VAL\r\n");
        return ERR_VAL;
    }

    cyw43_arch_lwip_check();

    if (rx_data->tot_len > 0) {
        char *p = (char*)(rx_data->payload);
        srv->cmd = p[0];

        (*(srv->cb))(arg);

        return server_data_tx(arg, conn_pcb);
    }

    pbuf_free(rx_data);

    printf("rx ERR_OK\r\n");

    return ERR_OK;
}

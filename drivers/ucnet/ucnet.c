// © 2026 StarIC Inc.
// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/poll.h>
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/sys/socket.h>
#include <zephyr/net/socket_service.h>
#include <unistd.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ucnet);

#include "ucuart.h"

#include "cb.h"
#include "cobs.h"
#include "cbor.h"

#include "log.h"

// Must match dts/bindings file - commas replaced with underscores
#define DT_DRV_COMPAT staric_ucnet

#define NET_PORT 9000

#define RX_CB_BUF_LEN  (1500)
#define PREFIX_BUF_LEN (256)

// uclog sends ping packets at this rate
#define UCLOG_PING_RATE_MS 500
// ping_timeout_timer will expire if no packets received in this time
#define PING_TIMEOUT_MS    (UCLOG_PING_RATE_MS * 2)

#ifdef CONFIG_UCNET_MDNS
#include <zephyr/net/dns_sd.h>

/* clang-format off */
    static const char service_txt[] = {
        "\x14" "serial_number=123456"
        "\x11" "description=P3260"
    };
/* clang-format on */

#if defined(CONFIG_UCNET_PROTOCOL_TCP)
DNS_SD_REGISTER_TCP_SERVICE(uclog, CONFIG_NET_HOSTNAME, "_uclog", "local", service_txt, NET_PORT);
#elif defined(CONFIG_UCNET_PROTOCOL_UDP)
DNS_SD_REGISTER_UDP_SERVICE(uclog, CONFIG_NET_HOSTNAME, "_uclog", "local", service_txt, NET_PORT);
#endif
#endif

static uint8_t ucnet_rx_buf[RX_CB_BUF_LEN];
static cb_t ucnet_rx_cb = CB_INIT(ucnet_rx_buf);

enum connected_state {
    WAITING_FOR_CONNECT,
    CONNECTED,
};

struct ucnet_config {
};
static struct ucnet_config config;

struct ucnet_data {
    // atomic_t tx_active;
    atomic_t connect_state;
    cb_t *tx_cb;
    uint8_t prefix_buf[PREFIX_BUF_LEN];
    size_t prefix_index;
    size_t prefix_len;
    const struct device *dev;

    K_KERNEL_STACK_MEMBER(stack_area, CONFIG_UCNET_TX_WQ_STACK_SIZE);
    struct k_work_q work_q;

    struct k_event event;
#if defined(CONFIG_UCNET_PROTOCOL_UDP)
    struct k_timer ping_timeout_timer;
#endif
};

static struct ucnet_data data = {
    // .tx_active = false,
    .connect_state = WAITING_FOR_CONNECT,
    .tx_cb = NULL,
    .prefix_index = 0U,
    .prefix_len = 0U,
};

static void socket_service_handler(struct net_socket_service_event *pev);

#if defined(CONFIG_UCNET_PROTOCOL_TCP)
static struct pollfd listening_socket_fd;
static struct pollfd client_socket_fd;

static void tcp_accept_handler(struct net_socket_service_event *pev);

NET_SOCKET_SERVICE_SYNC_DEFINE_STATIC(service_accept, tcp_accept_handler, 1);
NET_SOCKET_SERVICE_SYNC_DEFINE_STATIC(service_tcp, socket_service_handler, 1);
#elif defined(CONFIG_UCNET_PROTOCOL_UDP)
static struct pollfd udp_socket_fd;

struct sockaddr_in udp_addr;

NET_SOCKET_SERVICE_SYNC_DEFINE_STATIC(service_udp, socket_service_handler, 1);
#endif

#if defined(CONFIG_UCNET_PROTOCOL_TCP)
static void connection_closed(int client)
{
    atomic_set(&data.connect_state, WAITING_FOR_CONNECT);
    client_socket_fd.fd = -1;

    data.prefix_index = 0;

    int ret = net_socket_service_register(&service_tcp, &client_socket_fd, 1, NULL);
    if (ret < 0) {
        LOG_ERR("Cannot register socket service handler (%d)", ret);
    }

    close(client);
}
#endif

static ssize_t sendall(int sock, const void *buf, size_t len)
{
	while (len) {
		ssize_t out_len = send(sock, buf, len, 0);

		if (out_len < 0) {
			return out_len;
		}

		buf = (const char *)buf + out_len;
		len -= out_len;
	}

	return 0;
}

static void ucnet_send(struct k_work *item)
{
    while (data.tx_cb && atomic_get(&data.connect_state) == CONNECTED) {
        size_t n = cb_peek_avail(data.tx_cb);
        if (data.prefix_len > data.prefix_index) {
            // Send prefix first
            n = data.prefix_len - data.prefix_index;
#if defined(CONFIG_UCNET_PROTOCOL_TCP)
            int n_sent = sendall(client_socket_fd.fd, &data.prefix_buf[data.prefix_index], n);
            if (n_sent == 0) {
                n_sent = n;
            }
#elif defined(CONFIG_UCNET_PROTOCOL_UDP)
            int n_sent = sendto(udp_socket_fd.fd, &data.prefix_buf[data.prefix_index], n, 0,
                    (struct sockaddr *)&udp_addr, sizeof(udp_addr));
#endif

            if (n_sent < 0) {
                LOG_ERR("send: %d", -errno);
#if defined(CONFIG_UCNET_PROTOCOL_TCP)
                LOG_ERR("Closing connection");
                connection_closed(client_socket_fd.fd);
#endif
                return;
            } else if (n_sent > 0) {
                data.prefix_index += n_sent;
            }
        } else if (n > 0) {
#if defined(CONFIG_UCNET_PROTOCOL_TCP)
            int n_sent = sendall(client_socket_fd.fd, cb_peek(data.tx_cb), n);
            if (n_sent == 0) {
                n_sent = n;
            }
#elif defined(CONFIG_UCNET_PROTOCOL_UDP)
            int n_sent = sendto(udp_socket_fd.fd, cb_peek(data.tx_cb), n, 0,
                    (struct sockaddr *)&udp_addr, sizeof(udp_addr));
#endif
            if (n_sent < 0) {
                LOG_ERR("send: %d", -errno);
#if defined(CONFIG_UCNET_PROTOCOL_TCP)
                LOG_ERR("Closing connection");
                connection_closed(client_socket_fd.fd);
#endif
                return;

            } else if (n_sent > 0) {
                cb_skip(data.tx_cb, n_sent);
            }
        } else {
            // Finished work
            return;
        }
    }
}
static K_WORK_DEFINE(ucnet_send_work, ucnet_send);

static int tx_schedule(const struct device *dev)
{
    // const struct ucnet_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    struct ucnet_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    // LOG_INFO("ucnet_tx_schedule %d %p", data->tx_active, data->tx_cb);

    if (data->tx_cb && atomic_get(&data->connect_state) == CONNECTED) {
        k_work_submit_to_queue(&data->work_q, &ucnet_send_work);
        return 0;
    } else {
        return -ENOTCONN;
    }
}

static int set_connect_prefix(const struct device *dev, const uint8_t *prefix, size_t pn)
{
    struct ucnet_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    if ((prefix != NULL) && (pn > 0)) {
        if (pn > sizeof(data->prefix_buf)) {
            LOG_ERROR("Prefix too long: %zu", pn);
            data->prefix_index = 0;
            data->prefix_len = 0;
        } else {
            memcpy(data->prefix_buf, prefix, pn);
            data->prefix_index = 0;
            data->prefix_len = pn;
        }
    } else {
        data->prefix_index = 0;
        data->prefix_len = 0;
    }

    return 0;
}

static int tx(const struct device *dev, const uint8_t *b, size_t n)
{
    struct ucnet_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    // LOG_INFO("ucnet_tx %s %p", dev->name, data->tx_cb);

    if (data->tx_cb == NULL) {
        return -EIO;
    }

    cb_write(data->tx_cb, b, n);
    return tx_schedule(dev);
}

static int tx_buffer(const struct device *dev, const uint8_t *b, size_t n)
{
    struct ucnet_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    if (data->tx_cb == NULL) {
        return -EIO;
    }

    // LOG_INFO("ucnet_tx_buffer");
    cb_write(data->tx_cb, b, n);
    return 0;
}

static int set_tx_cb(const struct device *dev, cb_t *cb)
{
    struct ucnet_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);
    data->tx_cb = cb;
    LOG_INFO("ucnet_set_tx_cb %p", data->tx_cb);
    return 0;
}

static void rx_start(const struct device *dev)
{
}

static void rx_stop(const struct device *dev)
{
}

static size_t rx_avail(const struct device *dev)
{
    const struct ucnet_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    (void)config; // TODO: use
    return cb_peek_avail(&ucnet_rx_cb);
}

static const uint8_t *rx_peek(const struct device *dev)
{
    const struct ucnet_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    (void)config; // TODO: use
    return cb_peek(&ucnet_rx_cb);
}

static void rx_skip(const struct device *dev, size_t n)
{
    const struct ucnet_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    (void)config; // TODO: use
    cb_skip(&ucnet_rx_cb, n);
}

static uint32_t wait_event(const struct device *dev, uint32_t mask, bool reset, k_timeout_t timeout)
{
    // It is OK for events to occur between k_event_wait and k_event_clear.
    // The data associated with these events will be picked up with the capture.
    // The client must ensure that before calling wait_event the call to
    // uart_rx_avail returns 0;
    struct ucnet_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);
    (void)reset;
    uint32_t r = k_event_wait(&data->event, mask, 0, timeout);
    if (r != 0) {
        k_event_clear(&data->event, r);
    }
    return r;
}

static int panic(const struct device *dev)
{
    (void)dev;
    // TODO Need to call interrupt handler like USB driver does
    return 0;
}

static const struct ucuart_driver_api ucnet_api = {
    .tx_no_wait = tx,
    .tx_buffer = tx_buffer,
    .tx_schedule = tx_schedule,
    .set_connect_prefix = set_connect_prefix,
    .set_tx_cb = set_tx_cb,
    .rx_start = rx_start,
    .rx_stop = rx_stop,
    .rx_avail = rx_avail,
    .rx_peek = rx_peek,
    .rx_skip = rx_skip,
    .wait_event = wait_event,
    .panic = panic,
};

#if defined(CONFIG_UCNET_PROTOCOL_UDP)
static void ping_timeout(struct k_timer *timer)
{
    struct ucnet_data *data = CONTAINER_OF(timer, struct ucnet_data, ping_timeout_timer);
    atomic_set(&data->connect_state, WAITING_FOR_CONNECT);
    data->prefix_index = 0;
    LOG_WARN("Ping timeout expired: Host disconnected");

#ifdef CONFIG_PM_DEVICE_RUNTIME
    int ret = pm_device_runtime_put(data->dev);
    if (ret < 0) {
        LOG_ERROR("Couldn't suspend pm state: %d", ret);
    }
#endif
}
#endif

static int receive_data(struct net_socket_service_event *pev, char *buf, size_t buflen)
{
    struct pollfd *pfd = &pev->event;
    int client = pfd->fd;
    struct sockaddr_in addr;
    char addr_str[INET_ADDRSTRLEN];
    socklen_t addrlen = sizeof(addr);
    int len;

    len = recvfrom(client, buf, buflen, 0, (struct sockaddr *)&addr, &addrlen);
    if (len <= 0) {
        if (len < 0) {
            LOG_ERR("recv: %d", -errno);
        }
#if defined(CONFIG_UCNET_PROTOCOL_TCP)
        connection_closed(client);

        inet_ntop(addr.sin_family, &addr.sin_addr, addr_str, sizeof(addr_str));
        LOG_INF("Connection from %s closed", addr_str);
#endif
    } else {
#if defined(CONFIG_UCNET_PROTOCOL_UDP)
        k_timer_start(&data.ping_timeout_timer, K_MSEC(PING_TIMEOUT_MS), K_NO_WAIT);

        if (atomic_get(&data.connect_state) == WAITING_FOR_CONNECT) {
            udp_addr = addr;
            LOG_INF("New data from %s", addr_str);
            atomic_set(&data.connect_state, CONNECTED);
        }
#endif
    }

    return len;
}

static void socket_service_handler(struct net_socket_service_event *pev)
{
    // New data to read
    size_t space_avail = cb_space_avail(&ucnet_rx_cb);
    if (space_avail > 0) {
        int n = receive_data(pev, cb_space(&ucnet_rx_cb), space_avail);
        if (n > 0) {
            cb_commit(&ucnet_rx_cb, n);

            k_event_post(&data.event, UCUART_EVT_RX);
        }
    }
}

#if defined(CONFIG_UCNET_PROTOCOL_TCP)
static void tcp_accept_handler(struct net_socket_service_event *pev)
{
    int client;
    int sock = pev->event.fd;
    char addr_str[INET_ADDRSTRLEN];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    client = accept(sock, (struct sockaddr *)&client_addr, &client_addr_len);
    if (client < 0) {
        LOG_ERR("accept: %d", -errno);
        return;
    }

    inet_ntop(client_addr.sin_family, &client_addr.sin_addr, addr_str, sizeof(addr_str));
    LOG_INF("Connection from %s (%d)", addr_str, client);

    client_socket_fd.fd = client;
    client_socket_fd.events = POLLIN;
    (void)atomic_cas(&data.connect_state, WAITING_FOR_CONNECT, CONNECTED);

    int ret = net_socket_service_register(&service_tcp, &client_socket_fd, 1, NULL);
    if (ret < 0) {
        LOG_ERR("Cannot register socket service handler (%d)", ret);
    }
}
#endif

static int setup_socket(struct sockaddr_in *addr, int proto)
{
    int sock;

    int type = (proto == IPPROTO_TCP) ? SOCK_STREAM : SOCK_DGRAM;

    sock = socket(AF_INET, type, proto);
    if (sock < 0) {
        LOG_ERR("socket: %d", -errno);
        return -errno;
    }

    if (bind(sock, (struct sockaddr *)addr, sizeof(*addr)) < 0) {
        LOG_ERR("bind: %d", -errno);
        close(sock);
        return -errno;
    }

    if (proto == IPPROTO_TCP) {
        if (listen(sock, 1) < 0) {
            LOG_ERR("listen: %d", -errno);
            close(sock);
            return -errno;
        }
    }

    return sock;
}

static int start_service(void)
{
    int sock;

#if defined(CONFIG_UCNET_PROTOCOL_TCP)
    listening_socket_fd.fd = -1;
    client_socket_fd.fd = -1;
#elif defined(CONFIG_UCNET_PROTOCOL_UDP)
    udp_socket_fd.fd = -1;
#endif

    // Restart rx/tx tasks

    // Configure socket
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr = NET_INADDR_ANY_INIT,
        .sin_port = htons(NET_PORT),
    };

    int proto = IS_ENABLED(CONFIG_UCNET_PROTOCOL_TCP) ? IPPROTO_TCP : IPPROTO_UDP;
    sock = setup_socket(&addr, proto);
    if (sock < 0) {
        LOG_ERR("Failed to setup tcp listening socket");
        return sock;
    }

    atomic_set(&data.connect_state, WAITING_FOR_CONNECT);

#if defined(CONFIG_UCNET_PROTOCOL_TCP)
    listening_socket_fd.fd = sock;
    listening_socket_fd.events = POLLIN;

    /* Register TCP listening socket to service handler */
    int ret = net_socket_service_register(&service_accept, &listening_socket_fd, 1, NULL);

#elif defined(CONFIG_UCNET_PROTOCOL_UDP)
    udp_socket_fd.fd = sock;
    udp_socket_fd.events = POLLIN;

    /* Register UDP socket to service handler */
    int ret = net_socket_service_register(&service_udp, &udp_socket_fd, 1, NULL);
#endif

    if (ret < 0) {
        LOG_ERR("Cannot register socket service handler (%d)", ret);
        close(sock);
        return ret;
    } else {
        LOG_INF("Socket service handler registered");
    }

    return 0;
}

static int ucnet_enable(const struct device *dev)
{
    // const struct ucnet_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    // struct ucnet_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    return start_service();
}

#ifdef CONFIG_PM_DEVICE
static void socket_list_deinit(struct pollfd *socket_fds, size_t num_fds)
{
    k_mutex_lock(&lock, K_FOREVER);
    for (size_t i = 0; i < num_fds; i++) {
        if (socket_fds[i].fd != -1) {
            close(socket_fds[i].fd);
            socket_fds[i].fd = -1;
        }
    }
    k_mutex_unlock(&lock);
}

static int stop_service(void)
{
    (void)net_socket_service_unregister(&service_tcp);
    socket_list_deinit(client_socket_fds, ARRAY_SIZE(client_socket_fds));
    (void)net_socket_service_unregister(&service_accept);
    socket_list_deinit(listening_socket_fds, ARRAY_SIZE(listening_socket_fds));

    return 0;
}

static int ucnet_disable(const struct device *dev)
{
    const struct ucnet_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);

    return stop_service();
}

static int uart_pm_action(const struct device *dev, enum pm_device_action action)
{
    LOG_DEBUG("pm_action: {enum:pm_device_action}%d", action);
    switch (action) {
    case PM_DEVICE_ACTION_RESUME:
        // Restart rx/tx tasks
        ucnet_enable(dev);
        break;

    case PM_DEVICE_ACTION_SUSPEND:
        // Stop rx/tx tasks - wait for rx/tx stopped to be complete
        ucnet_disable(dev);
        break;

    case PM_DEVICE_ACTION_TURN_ON:
    case PM_DEVICE_ACTION_TURN_OFF:
        break;

    default:
        return -ENOTSUP;
    }
    return 0;
}
#endif

static int ucnet_init(const struct device *dev)
{
    struct ucnet_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    data->dev = dev;

    k_work_queue_init(&data->work_q);
    k_work_queue_start(&data->work_q, data->stack_area, K_THREAD_STACK_SIZEOF(data->stack_area),
            CONFIG_UCNET_TX_WQ_THREAD_PRIORITY, NULL);
    if (k_thread_name_set(&data->work_q.thread, "UCNET TX WQ") != 0) {
        // Couldn't set thread name
    }

    k_event_init(&data->event);
#if defined(CONFIG_UCNET_PROTOCOL_UDP)
    k_timer_init(&data->ping_timeout_timer, ping_timeout, NULL);
#endif

#ifdef CONFIG_PM_DEVICE_RUNTIME
    /* Start suspended */
    ucnet_disable(dev);

    pm_device_init_suspended(dev);

    ret = pm_device_runtime_enable(dev);
    if (ret != 0) {
        LOG_ERROR("Couldn't enable pm device runtime: %d", ret);
        return ret;
    }
#else
    // Start enabled
    return ucnet_enable(dev);
#endif
}

/* clang-format off */
#define UCNET_DEFINE(i)                                                                           \
                                                                                                   \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(i, uart_pm_action);                                                   \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(i, ucnet_init, PM_DEVICE_DT_INST_GET(i), &data, &config,          \
            POST_KERNEL, CONFIG_UCNET_INIT_PRIORITY, &ucnet_api);

/* clang-format on */

DT_INST_FOREACH_STATUS_OKAY(UCNET_DEFINE)

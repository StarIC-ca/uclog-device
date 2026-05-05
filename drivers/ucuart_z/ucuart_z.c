// © 2026 StarIC Inc.
// SPDX-License-Identifier: Apache-2.0

#include "ucuart.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>

#include "cb.h"
#include "cobs.h"
#include "cbor.h"

#include "log.h"

// Must match dts/bindings file - commas replaced with underscores
#define DT_DRV_COMPAT staric_ucuart_z

#define RX_CB_BUF_LEN (1024)

#define RX_TEMP_BUF_LEN (256)
#define NUM_RX_BUFS     (3)

// uclog sends ping packets at this rate
#define UCLOG_PING_RATE_MS 500
// ping_timeout_timer will expire if no packets received in this time
#define PING_TIMEOUT_MS    (UCLOG_PING_RATE_MS * 2)

struct ucuart_config {
    const struct device *uart;
    cb_t *rx_cb;
    uint32_t current_speed;
    struct gpio_dt_spec wakeup_gpio;
};

struct ucuart_data {
    atomic_t tx_active;
    atomic_t host_ready;
    cb_t *tx_cb;
#ifdef CONFIG_UART_ASYNC_API
    uint8_t *rx_temp_buf;
    size_t rx_temp_buf_index;
#endif
    uint8_t prefix_buf[RX_TEMP_BUF_LEN];
    size_t prefix_index;
    size_t prefix_len;
    const struct device *dev;

    struct k_event event;
    struct k_timer ping_timeout_timer;
    uint8_t ppi;
    struct gpio_callback wakeup_gpio_cb;
    struct k_work_delayable wakeup_work;
};

typedef enum {
    UCUART_ERROR_NONE = 0,
    UCUART_ERROR_TX_DMA = 1 << 0,
    UCUART_ERROR_RX_DMA = 1 << 1,
    UCUART_ERROR_OR = 1 << 2,
    UCUART_ERROR_FRAMING = 1 << 3,
    UCUART_ERROR_NOISE = 1 << 4,
} uart_error_t;

#if defined(CONFIG_UART_ASYNC_API)
#define UART_ASYNC_RX_TIMEOUT_US 100
#elif defined(CONFIG_UART_INTERRUPT_DRIVEN)
#else
#error "ASYNC or INTERRUPT API is required"
#endif

#if defined(CONFIG_UART_ASYNC_API)
static void uart_event_handler(const struct device *dev, struct uart_event *evt, void *user_data)
{
    struct ucuart_data *data = (struct ucuart_data *)user_data;
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(data->dev, config);
    switch (evt->type) {
    case UART_TX_DONE: {
        if (data->prefix_len > data->prefix_index) {
            // prefix was sent
            data->prefix_index = data->prefix_len;
        } else {
            cb_skip(data->tx_cb, evt->data.tx.len);
        }

        // If there is more data then send it now
        size_t n = cb_peek_avail(data->tx_cb);
        if (n > 0 && atomic_get(&data->host_ready)) {
            int ret = uart_tx(dev, cb_peek(data->tx_cb), n, SYS_FOREVER_US);
            if (ret != 0) {
                LOG_ERROR("uart_tx() failed, ret %d", ret);
                atomic_set(&data->tx_active, false);
            }
        } else {
            atomic_set(&data->tx_active, false);
        }
        break;
    }
    case UART_TX_ABORTED:
        LOG_ERROR("UART_TX_ABORTED!");
        break;
    case UART_RX_RDY: {
        size_t n = evt->data.rx.len;
        uint8_t *p = &evt->data.rx.buf[evt->data.rx.offset];

        // copy data received into circular buffer
        size_t na = cb_write_avail(config->rx_cb);
        if (n > na) {
            LOG_ERROR("dropping rx data n:%u", n - na);
            n = na;
        }
        cb_write(config->rx_cb, p, n);

        k_timer_start(&data->ping_timeout_timer, K_MSEC(PING_TIMEOUT_MS), K_NO_WAIT);

        atomic_set(&data->host_ready, true);

        k_event_post(&data->event, UCUART_EVT_RX);
        break;
    }
    case UART_RX_BUF_REQUEST:
        data->rx_temp_buf_index += RX_TEMP_BUF_LEN;
        data->rx_temp_buf_index %= (NUM_RX_BUFS * RX_TEMP_BUF_LEN);
        uart_rx_buf_rsp(dev, &data->rx_temp_buf[data->rx_temp_buf_index], RX_TEMP_BUF_LEN);
        break;
    case UART_RX_BUF_RELEASED:
        break;
    case UART_RX_DISABLED: {
        cb_reset(config->rx_cb);
        atomic_set(&data->host_ready, false);
        atomic_set(&data->tx_active, false);
        data->rx_temp_buf_index = 0;

        LOG_WARN("UART_RX_DISABLED!");
        break;
    }
    case UART_RX_STOPPED:
        LOG_WARN("UART_RX_STOPPED: {enum:uart_rx_stop_reason}%d", evt->data.rx_stop.reason);
        break;
    }
}
#elif defined(CONFIG_UART_INTERRUPT_DRIVEN)
static void uart_isr(const struct device *uart_dev, void *user_data)
{
    struct ucuart_data *data = (struct ucuart_data *)user_data;
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(data->dev, config);

    while (uart_irq_update(uart_dev) && uart_irq_is_pending(uart_dev)) {
        if (uart_irq_rx_ready(uart_dev)) {
            // New data to read
            size_t space_avail = cb_space_avail(config->rx_cb);
            if (space_avail > 0) {
                int n = uart_fifo_read(uart_dev, cb_space(config->rx_cb), space_avail);
                if (n < 0) {
                    LOG_FATAL("uart_fifo_read returned %d", n);
                } else if (n > 0) {
                    cb_commit(config->rx_cb, n);

                    k_timer_start(&data->ping_timeout_timer, K_MSEC(PING_TIMEOUT_MS), K_NO_WAIT);

                    atomic_set(&data->host_ready, true);

                    k_event_post(&data->event, UCUART_EVT_RX);
                }
            } else {
                uint8_t discard_buf[32];
                int n = uart_fifo_read(uart_dev, discard_buf, sizeof(discard_buf));
                if (n < 0) {
                    LOG_FATAL("uart_fifo_read returned %d", n);
                } else if (n > 0) {
                    LOG_ERROR("Discarded %d bytes (rx buf full)", n);
                }
            }
        }

        if (uart_irq_tx_ready(uart_dev)) {
            if (data->prefix_len > data->prefix_index) {
                // Send prefix first
                size_t n = data->prefix_len - data->prefix_index;
                int n_sent = uart_fifo_fill(uart_dev, &data->prefix_buf[data->prefix_index], n);
                if (n_sent < 0) {
                    LOG_FATAL("uart_fifo_fill returned %d", n);
                } else if (n_sent > 0) {
                    data->prefix_index += n_sent;
                }
            } else {
                size_t n = cb_peek_avail(data->tx_cb);
                if (n > 0) {
                    int n_sent = uart_fifo_fill(uart_dev, cb_peek(data->tx_cb), n);
                    if (n_sent < 0) {
                        LOG_FATAL("uart_fifo_fill returned %d", n);
                    } else if (n_sent > 0) {
                        cb_skip(data->tx_cb, n_sent);
                    }
                } else {
                    uart_irq_tx_disable(uart_dev);
                    atomic_set(&data->tx_active, false);
                }
            }
        }
    }
}
#endif

static int tx_schedule(const struct device *dev, const uint8_t* prefix, size_t pn)
{
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    // LOG_INFO("ucuart_tx_schedule %d %p", data->tx_active, data->tx_cb);

    if (data->tx_cb && atomic_get(&data->host_ready)) {
        bool got = atomic_cas(&data->tx_active, false, true);
        if (got) {
            size_t n = cb_peek_avail(data->tx_cb);
#if defined(CONFIG_UART_ASYNC_API)
            if ((prefix != NULL) && (pn > 0)) {
                data->prefix_index = 0;
                data->prefix_len = pn;
                int ret = uart_tx(config->uart, prefix, pn, SYS_FOREVER_US);
                if (ret != 0) {
                    LOG_ERROR("uart_tx() failed, ret %d", ret);
                    atomic_set(&data->tx_active, false);
                }
            } else if (n > 0) {
                int ret = uart_tx(config->uart, cb_peek(data->tx_cb), n, SYS_FOREVER_US);
                if (ret != 0) {
                    LOG_ERROR("uart_tx() failed, ret %d", ret);
                    atomic_set(&data->tx_active, false);
                }
            } else {
                atomic_set(&data->tx_active, false);
            }
#else
            if ((prefix != NULL) && (pn > 0)) {
                if (pn > sizeof(data->prefix_buf)) {
                    LOG_ERROR("Prefix too long: %zu", pn);
                    atomic_set(&data->tx_active, false);
                    return -EINVAL;
                } else {
                    memcpy(data->prefix_buf, prefix, pn);
                    data->prefix_index = 0;
                    data->prefix_len = pn;
                    uart_irq_tx_enable(config->uart);
                }
            } else if (n > 0) {
                // Just enable TX. Writing into FIFO is done in ISR.
                uart_irq_tx_enable(config->uart);
            } else {
                atomic_set(&data->tx_active, false);
            }
#endif
        }
    }
    return 0;
}

static int tx(const struct device *dev, const uint8_t *b, size_t n)
{
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    // LOG_INFO("ucuart_tx %s %p", dev->name, data->tx_cb);

    if (data->tx_cb == NULL) {
        return -EIO;
    }

    cb_write(data->tx_cb, b, n);
    return tx_schedule(dev, NULL, 0);
}

static int tx_buffer(const struct device *dev, const uint8_t *b, size_t n)
{
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    if (data->tx_cb == NULL) {
        return -EIO;
    }

    // LOG_INFO("ucuart_tx_buffer");
    cb_write(data->tx_cb, b, n);
    return 0;
}

static int set_tx_cb(const struct device *dev, cb_t *cb)
{
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);
    data->tx_cb = cb;
    LOG_INFO("ucuart_set_tx_cb %p", data->tx_cb);
    return 0;
}

static void rx_start(const struct device *dev)
{
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);
    (void)data;
}

static void rx_stop(const struct device *dev)
{
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);
    (void)data;
}

static size_t rx_avail(const struct device *dev)
{
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    return cb_peek_avail(config->rx_cb);
}

static const uint8_t *rx_peek(const struct device *dev)
{
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    return cb_peek(config->rx_cb);
}

static void rx_skip(const struct device *dev, size_t n)
{
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    cb_skip(config->rx_cb, n);
}

static uint32_t wait_event(const struct device *dev, uint32_t mask, bool reset, k_timeout_t timeout)
{
    // It is OK for events to occur between k_event_wait and k_event_clear.
    // The data associated with these events will be picked up with the capture.
    // The client must ensure that before calling wait_event the call to
    // uart_rx_avail returns 0;
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);
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

static const struct ucuart_driver_api ucuart_api = {
    .tx_no_wait = tx,
    .tx_buffer = tx_buffer,
    .tx_schedule = tx_schedule,
    .set_tx_cb = set_tx_cb,
    .rx_start = rx_start,
    .rx_stop = rx_stop,
    .rx_avail = rx_avail,
    .rx_peek = rx_peek,
    .rx_skip = rx_skip,
    .wait_event = wait_event,
    .panic = panic,
};

void ping_timeout(struct k_timer *timer)
{
    struct ucuart_data *data = CONTAINER_OF(timer, struct ucuart_data, ping_timeout_timer);
    atomic_set(&data->host_ready, false);
    LOG_WARN("Ping timeout expired: Host disconnected");

#ifdef CONFIG_PM_DEVICE_RUNTIME
    int ret = pm_device_runtime_put(data->dev);
    if (ret < 0) {
        LOG_ERROR("Couldn't suspend pm state: %d", ret);
    }
#endif
}

static void wakeup(struct k_work *item)
{
    LOG_INFO("Wakeup triggered");
#ifdef CONFIG_PM_DEVICE_RUNTIME
    struct k_work_delayable *dwork = k_work_delayable_from_work(item);
    struct ucuart_data *data = CONTAINER_OF(dwork, struct ucuart_data, wakeup_work);
    int ret = pm_device_runtime_get(data->dev);
    if (ret < 0) {
        LOG_ERROR("Couldn't resume pm state: %d", ret);
    }
#endif
}

void wakeup_gpio_triggered(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // dev is the gpio dev, so need to get our dev
    struct ucuart_data *data = CONTAINER_OF(cb, struct ucuart_data, wakeup_gpio_cb);
    const struct device *our_dev = data->dev;
    const struct ucuart_config *config = our_dev->config;

    gpio_pin_interrupt_configure_dt(&config->wakeup_gpio, GPIO_INT_DISABLE);

    // We can do pm_device_runtime_get() in ISR, but we actually want to delay the uart enable
    // so we aren't starting RX in the middle of the xfer that woke us
    k_work_schedule(&data->wakeup_work, K_MSEC(1));
}



static int ucuart_enable(const struct device *dev)
{
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);

    // Restart rx/tx tasks
#if defined(CONFIG_UART_ASYNC_API)
    int ret = uart_rx_enable(config->uart, &data->rx_temp_buf[data->rx_temp_buf_index], RX_TEMP_BUF_LEN, UART_ASYNC_RX_TIMEOUT_US);
    if (ret != 0) {
        LOG_ERROR("uart_rx_enable() failed, ret %d", ret);
        return -EIO;
    }
#elif defined(CONFIG_UART_INTERRUPT_DRIVEN)
    uart_irq_rx_enable(config->uart);
#endif

    k_timer_start(&data->ping_timeout_timer, K_MSEC(PING_TIMEOUT_MS), K_NO_WAIT);

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int rx_abort(const struct device *dev)
{
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);
    int ret;

    ret = uart_rx_disable(config->uart);
    if (ret == 0) {
        // success
    } else if (ret == -EFAULT) {
        // no active rx
        cb_reset(config->rx_cb);
        atomic_set(&data->host_ready, false);
        atomic_set(&data->tx_active, false);
#ifdef CONFIG_UART_ASYNC_API
        data->rx_temp_buf_index = 0;
#endif
        ret = 0;
    } else {
        LOG_ERROR("uart_rx_disable failed: %d", ret);
        return ret;
    }

    return ret;
}

static int ucuart_disable(const struct device *dev)
{
    int ret = 0;
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);

    rx_abort(dev);

    if (config->wakeup_gpio.port != NULL) {
        // Set GPIO wakeup trigger
        int ret = gpio_pin_configure_dt(&config->wakeup_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERROR("Unable to configure GPIO pin");
            return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&config->wakeup_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERROR("Unable to configure interrupt on pin");
        }
    }
    return ret;
}

static int uart_pm_action(const struct device *dev, enum pm_device_action action)
{
    LOG_DEBUG("pm_action: {enum:pm_device_action}%d", action);
    switch (action) {
    case PM_DEVICE_ACTION_RESUME:
        // Restart rx/tx tasks
        ucuart_enable(dev);
        break;

    case PM_DEVICE_ACTION_SUSPEND:
        // Stop rx/tx tasks - wait for rx/tx stopped to be complete
        ucuart_disable(dev);
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

static int ucuart_init(const struct device *dev)
{
    const struct ucuart_config *config = ZEPHYR_DEVICE_MEMBER(dev, config);
    struct ucuart_data *data = ZEPHYR_DEVICE_MEMBER(dev, data);
    int ret;

    data->dev = dev;

    k_event_init(&data->event);
    k_timer_init(&data->ping_timeout_timer, ping_timeout, NULL);

    // Configure UART
    struct uart_config uart_cfg = {.baudrate = config->current_speed,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

    ret = uart_configure(config->uart, &uart_cfg);
    if (ret != 0) {
        LOG_ERROR("uart_configure() failed, ret %d", ret);
        return -EIO;
    }
#if defined(CONFIG_UART_ASYNC_API)
    ret = uart_callback_set(config->uart, uart_event_handler, (void *)data);
#elif defined(CONFIG_UART_INTERRUPT_DRIVEN)
    uart_irq_rx_disable(config->uart);
    uart_irq_tx_disable(config->uart);

    ret = uart_irq_callback_user_data_set(config->uart, uart_isr, (void *)data);
#else
    ret = -ENOSYS;
#endif
    if (ret != 0) {
        LOG_ERROR("Couldn't set callback, ret %d", ret);
        return ret;
    }

    if (config->wakeup_gpio.port != NULL) {
        // Set up wakeup trigger callback
        gpio_init_callback(&data->wakeup_gpio_cb, wakeup_gpio_triggered,
                (gpio_port_pins_t)BIT(config->wakeup_gpio.pin));

        ret = gpio_add_callback(config->wakeup_gpio.port, &data->wakeup_gpio_cb);
        if (ret < 0) {
            LOG_ERROR("Could not set gpio callback");
            return ret;
        }
        k_work_init_delayable(&data->wakeup_work, wakeup);
    }

#ifdef CONFIG_PM_DEVICE_RUNTIME
    /* Start suspended */
    ucuart_disable(dev);

    pm_device_init_suspended(dev);

    ret = pm_device_runtime_enable(dev);
    if (ret != 0) {
        LOG_ERROR("Couldn't enable pm device runtime: %d", ret);
        return ret;
    }
#else
    // Start enabled
    ucuart_enable(dev);
#endif

    return 0;
}

#if defined(CONFIG_SOC_FAMILY_STM32) && defined(CONFIG_NOCACHE_MEMORY)
#define NOCACHE_ATTR __nocache
#else
#define NOCACHE_ATTR
#endif

#ifdef CONFIG_UART_ASYNC_API
#define UART_TEMP_BUF_INIT()                \
    static uint8_t ucuart_rx_temp_buf##i[NUM_RX_BUFS * RX_TEMP_BUF_LEN] NOCACHE_ATTR
#define UART_ASYNC_DATA_INIT()              \
    .rx_temp_buf_index = 0U,                \
    .rx_temp_buf = ucuart_rx_temp_buf##i,
#else
#define UART_TEMP_BUF_INIT() /* Not used */
#define UART_ASYNC_DATA_INIT() /* Not used */
#endif

/* clang-format off */
#define UCUART_DEFINE(i)                                                                           \
                                                                                                   \
    static uint8_t ucuart_rx_buf##i[RX_CB_BUF_LEN];                                                \
    static cb_t ucuart_rx_cb##i = CB_INIT(ucuart_rx_buf##i);                                       \
                                                                                                   \
    UART_TEMP_BUF_INIT();                                                                          \
                                                                                                   \
    static const struct ucuart_config config##i = {                                                \
        .uart = DEVICE_DT_GET(DT_INST_BUS(i)),                                                     \
        .rx_cb = &ucuart_rx_cb##i,                                                                 \
        .current_speed = DT_INST_PROP(i, current_speed),                                           \
        .wakeup_gpio = GPIO_DT_SPEC_INST_GET_OR(i, wakeup_gpios, {0}),                             \
    };                                                                                             \
                                                                                                   \
    static struct ucuart_data data##i = {                                                          \
        .tx_active = false,                                                                        \
        .host_ready = false,                                                                       \
        .tx_cb = NULL,                                                                             \
        .prefix_index = 0U,                                                                        \
        .prefix_len = 0U,                                                                          \
        UART_ASYNC_DATA_INIT()                                                                     \
    };                                                                                             \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(i, uart_pm_action);                                                   \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(i, ucuart_init, PM_DEVICE_DT_INST_GET(i), &data##i, &config##i,          \
            PRE_KERNEL_1, CONFIG_UCUART_Z_INIT_PRIORITY, &ucuart_api);

/* clang-format on */

DT_INST_FOREACH_STATUS_OKAY(UCUART_DEFINE)

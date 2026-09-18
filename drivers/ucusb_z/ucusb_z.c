/*
 * Copyright (c) 2026 StarIC Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief ucLog over USB CDC ACM, on the Zephyr USB device stack
 *
 * The host sees a plain CDC ACM serial port. The data is the ucLog byte stream: COBS frames
 * delimited by 0x00, the same as ucuart_z and ucnet carry.
 *
 * TX is zero copy. Each IN transfer is a net_buf that points into the ucLog TX ring buffer
 * (cb_peek()), and with CONFIG_UDC_STM32_DMA the OTG_HS core DMAs it straight out of the ring.
 * The STM32 OTG DMA fails silently unless a transfer starts on a word boundary and is a multiple
 * of 4 bytes long (found on the a43 STM32H750 ucLog USB driver; assumed to hold on the N6 too).
 * CONFIG_UC_LOG_TX_ALIGN4 guarantees that: the ring is word aligned and every write into it is
 * zero padded to 4 bytes, so every linear run of the ring qualifies. The 0x00 padding sits
 * between frames and the host ignores it.
 *
 * Threads: the USB stack calls the class API from its own cooperative thread. Everything that
 * moves the ring read index or starts an IN transfer runs in this driver's TX workqueue, so
 * tx_schedule() is safe from any context, including ISRs.
 *
 * Host connection: like ucuart_z, the host counts as connected while it keeps sending (the ucLog
 * client pings every 500 ms). The connect prefix (device info frame) is sent first on every new
 * connection.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usb_cdc.h>
#include <zephyr/logging/log.h>

#include "ucuart.h"
#include "cb.h"
#include "log.h"

LOG_MODULE_REGISTER(ucusb_z, CONFIG_UCUSB_Z_LOG_LEVEL);

#define DT_DRV_COMPAT staric_ucusb_z

#if !defined(CONFIG_UC_LOG_TX_ALIGN4)
#error "ucusb_z needs CONFIG_UC_LOG_TX_ALIGN4=y (patches/uclog-log-tx-align4.patch)"
#endif

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "Only one ucusb_z instance is supported");
BUILD_ASSERT(DT_INST_ON_BUS(0, usb), "ucusb_z node must be a child of the USB device controller");
BUILD_ASSERT((CONFIG_UCUSB_Z_TX_MAX_SIZE % 512) == 0,
        "UCUSB_Z_TX_MAX_SIZE must be a multiple of 512");

#define BULK_MPS_HS 512U
#define BULK_MPS_FS 64U
#define INT_EP_MPS  16U

// OUT transfer buffer. With DMA it must be a multiple of the MPS at both speeds.
#define RX_XFER_LEN    1024U
// ucLog server reads host frames from here
#define RX_CB_BUF_LEN  2048U
#define PREFIX_BUF_LEN 256U

// uclog sends ping packets at this rate
#define UCLOG_PING_RATE_MS 500
// the host is considered gone if nothing is received in this time
#define PING_TIMEOUT_MS    (UCLOG_PING_RATE_MS * 2)

// data->state bits
enum {
    STATE_ENABLED,    // USB configuration selected by the host
    STATE_HOST_READY, // ucLog client is sending
    STATE_TX_BUSY,    // an IN transfer is queued on the endpoint
    STATE_TX_DONE,    // that transfer completed, waiting for the work item to retire it
    STATE_PREFIX,     // connect prefix still to be sent for this connection
};

struct ucusb_desc {
    struct usb_association_descriptor iad;
    struct usb_if_descriptor if0;
    struct cdc_header_descriptor if0_header;
    struct cdc_cm_descriptor if0_cm;
    struct cdc_acm_descriptor if0_acm;
    struct cdc_union_descriptor if0_union;
    struct usb_ep_descriptor if0_int_ep;
    struct usb_if_descriptor if1;
    struct usb_ep_descriptor if1_in_ep;
    struct usb_ep_descriptor if1_out_ep;
#if USBD_SUPPORTS_HIGH_SPEED
    struct usb_ep_descriptor if0_hs_int_ep;
    struct usb_ep_descriptor if1_hs_in_ep;
    struct usb_ep_descriptor if1_hs_out_ep;
#endif
    struct usb_desc_header nil_desc;
};

struct ucusb_data {
    const struct device *dev;
    struct usbd_class_data *c_data;
    atomic_t state;

    cb_t *tx_cb;
    // what the queued IN transfer covers, owned by the TX workqueue
    size_t tx_len;
    bool tx_is_prefix;
    uint32_t tx_errors;

    uint8_t rx_buf[RX_CB_BUF_LEN];
    cb_t rx_cb;

    size_t prefix_len;

    struct cdc_acm_line_coding line_coding;
    uint16_t line_state;

    K_KERNEL_STACK_MEMBER(stack_area, CONFIG_UCUSB_Z_TX_WQ_STACK_SIZE);
    struct k_work_q work_q;
    struct k_work tx_work;

    struct k_event event;
    struct k_timer ping_timeout_timer;
};

// The IN transfer net_bufs carry no data of their own, they point into the ucLog ring or at
// prefix_buf
NET_BUF_POOL_DEFINE(ucusb_in_pool, 1, 0, sizeof(struct udc_buf_info), NULL);
UDC_BUF_POOL_DEFINE(ucusb_out_pool, 1, RX_XFER_LEN, sizeof(struct udc_buf_info), NULL);

// Word aligned and zero padded to 4 bytes, like the ucLog ring
static uint8_t prefix_buf[PREFIX_BUF_LEN] __aligned(32);

static struct ucusb_desc ucusb_desc = {
    .iad =
            {
                .bLength = sizeof(struct usb_association_descriptor),
                .bDescriptorType = USB_DESC_INTERFACE_ASSOC,
                .bFirstInterface = 0,
                .bInterfaceCount = 0x02,
                .bFunctionClass = USB_BCC_CDC_CONTROL,
                .bFunctionSubClass = ACM_SUBCLASS,
                .bFunctionProtocol = 0,
                .iFunction = 0,
            },
    .if0 =
            {
                .bLength = sizeof(struct usb_if_descriptor),
                .bDescriptorType = USB_DESC_INTERFACE,
                .bInterfaceNumber = 0,
                .bAlternateSetting = 0,
                .bNumEndpoints = 1,
                .bInterfaceClass = USB_BCC_CDC_CONTROL,
                .bInterfaceSubClass = ACM_SUBCLASS,
                .bInterfaceProtocol = 0,
                .iInterface = 0,
            },
    .if0_header =
            {
                .bFunctionLength = sizeof(struct cdc_header_descriptor),
                .bDescriptorType = USB_DESC_CS_INTERFACE,
                .bDescriptorSubtype = HEADER_FUNC_DESC,
                .bcdCDC = sys_cpu_to_le16(USB_SRN_1_1),
            },
    .if0_cm =
            {
                .bFunctionLength = sizeof(struct cdc_cm_descriptor),
                .bDescriptorType = USB_DESC_CS_INTERFACE,
                .bDescriptorSubtype = CALL_MANAGEMENT_FUNC_DESC,
                .bmCapabilities = 0,
                .bDataInterface = 1,
            },
    .if0_acm =
            {
                .bFunctionLength = sizeof(struct cdc_acm_descriptor),
                .bDescriptorType = USB_DESC_CS_INTERFACE,
                .bDescriptorSubtype = ACM_FUNC_DESC,
                // SET/GET_LINE_CODING and SET_CONTROL_LINE_STATE (CDC PSTN 5.3.2)
                .bmCapabilities = BIT(1),
            },
    .if0_union =
            {
                .bFunctionLength = sizeof(struct cdc_union_descriptor),
                .bDescriptorType = USB_DESC_CS_INTERFACE,
                .bDescriptorSubtype = UNION_FUNC_DESC,
                .bControlInterface = 0,
                .bSubordinateInterface0 = 1,
            },
    // Required by CDC ACM hosts, never used
    .if0_int_ep =
            {
                .bLength = sizeof(struct usb_ep_descriptor),
                .bDescriptorType = USB_DESC_ENDPOINT,
                .bEndpointAddress = 0x81,
                .bmAttributes = USB_EP_TYPE_INTERRUPT,
                .wMaxPacketSize = sys_cpu_to_le16(INT_EP_MPS),
                .bInterval = USB_FS_INT_EP_INTERVAL(10000U),
            },
    .if1 =
            {
                .bLength = sizeof(struct usb_if_descriptor),
                .bDescriptorType = USB_DESC_INTERFACE,
                .bInterfaceNumber = 1,
                .bAlternateSetting = 0,
                .bNumEndpoints = 2,
                .bInterfaceClass = USB_BCC_CDC_DATA,
                .bInterfaceSubClass = 0,
                .bInterfaceProtocol = 0,
                .iInterface = 0,
            },
    .if1_in_ep =
            {
                .bLength = sizeof(struct usb_ep_descriptor),
                .bDescriptorType = USB_DESC_ENDPOINT,
                .bEndpointAddress = 0x82,
                .bmAttributes = USB_EP_TYPE_BULK,
                .wMaxPacketSize = sys_cpu_to_le16(BULK_MPS_FS),
                .bInterval = 0,
            },
    .if1_out_ep =
            {
                .bLength = sizeof(struct usb_ep_descriptor),
                .bDescriptorType = USB_DESC_ENDPOINT,
                .bEndpointAddress = 0x01,
                .bmAttributes = USB_EP_TYPE_BULK,
                .wMaxPacketSize = sys_cpu_to_le16(BULK_MPS_FS),
                .bInterval = 0,
            },
#if USBD_SUPPORTS_HIGH_SPEED
    .if0_hs_int_ep =
            {
                .bLength = sizeof(struct usb_ep_descriptor),
                .bDescriptorType = USB_DESC_ENDPOINT,
                .bEndpointAddress = 0x81,
                .bmAttributes = USB_EP_TYPE_INTERRUPT,
                .wMaxPacketSize = sys_cpu_to_le16(INT_EP_MPS),
                .bInterval = USB_HS_INT_EP_INTERVAL(10000U),
            },
    .if1_hs_in_ep =
            {
                .bLength = sizeof(struct usb_ep_descriptor),
                .bDescriptorType = USB_DESC_ENDPOINT,
                .bEndpointAddress = 0x82,
                .bmAttributes = USB_EP_TYPE_BULK,
                .wMaxPacketSize = sys_cpu_to_le16(BULK_MPS_HS),
                .bInterval = 0,
            },
    .if1_hs_out_ep =
            {
                .bLength = sizeof(struct usb_ep_descriptor),
                .bDescriptorType = USB_DESC_ENDPOINT,
                .bEndpointAddress = 0x01,
                .bmAttributes = USB_EP_TYPE_BULK,
                .wMaxPacketSize = sys_cpu_to_le16(BULK_MPS_HS),
                .bInterval = 0,
            },
#endif
    .nil_desc =
            {
                .bLength = 0,
                .bDescriptorType = 0,
            },
};

static const struct usb_desc_header *ucusb_fs_desc[] = {
    (struct usb_desc_header *)&ucusb_desc.iad,
    (struct usb_desc_header *)&ucusb_desc.if0,
    (struct usb_desc_header *)&ucusb_desc.if0_header,
    (struct usb_desc_header *)&ucusb_desc.if0_cm,
    (struct usb_desc_header *)&ucusb_desc.if0_acm,
    (struct usb_desc_header *)&ucusb_desc.if0_union,
    (struct usb_desc_header *)&ucusb_desc.if0_int_ep,
    (struct usb_desc_header *)&ucusb_desc.if1,
    (struct usb_desc_header *)&ucusb_desc.if1_in_ep,
    (struct usb_desc_header *)&ucusb_desc.if1_out_ep,
    (struct usb_desc_header *)&ucusb_desc.nil_desc,
};

#if USBD_SUPPORTS_HIGH_SPEED
static const struct usb_desc_header *ucusb_hs_desc[] = {
    (struct usb_desc_header *)&ucusb_desc.iad,
    (struct usb_desc_header *)&ucusb_desc.if0,
    (struct usb_desc_header *)&ucusb_desc.if0_header,
    (struct usb_desc_header *)&ucusb_desc.if0_cm,
    (struct usb_desc_header *)&ucusb_desc.if0_acm,
    (struct usb_desc_header *)&ucusb_desc.if0_union,
    (struct usb_desc_header *)&ucusb_desc.if0_hs_int_ep,
    (struct usb_desc_header *)&ucusb_desc.if1,
    (struct usb_desc_header *)&ucusb_desc.if1_hs_in_ep,
    (struct usb_desc_header *)&ucusb_desc.if1_hs_out_ep,
    (struct usb_desc_header *)&ucusb_desc.nil_desc,
};
#endif

static struct ucusb_data data = {
    .line_coding = {sys_cpu_to_le32(115200), 0, 0, 8},
};

static bool is_hs(struct usbd_class_data *const c_data)
{
    return USBD_SUPPORTS_HIGH_SPEED && usbd_bus_speed(usbd_class_get_ctx(c_data)) == USBD_SPEED_HS;
}

static uint8_t bulk_in_ep(struct usbd_class_data *const c_data)
{
#if USBD_SUPPORTS_HIGH_SPEED
    if (is_hs(c_data)) {
        return ucusb_desc.if1_hs_in_ep.bEndpointAddress;
    }
#endif
    return ucusb_desc.if1_in_ep.bEndpointAddress;
}

static uint8_t bulk_out_ep(struct usbd_class_data *const c_data)
{
#if USBD_SUPPORTS_HIGH_SPEED
    if (is_hs(c_data)) {
        return ucusb_desc.if1_hs_out_ep.bEndpointAddress;
    }
#endif
    return ucusb_desc.if1_out_ep.bEndpointAddress;
}

static void tx_kick(void)
{
    (void)k_work_submit_to_queue(&data.work_q, &data.tx_work);
}

// TX workqueue. The only place that starts IN transfers or moves the ring read index.
static void tx_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (atomic_test_and_clear_bit(&data.state, STATE_TX_DONE)) {
        if (data.tx_is_prefix) {
            atomic_clear_bit(&data.state, STATE_PREFIX);
        } else {
            cb_skip(data.tx_cb, data.tx_len);
        }
        atomic_clear_bit(&data.state, STATE_TX_BUSY);
    }

    if (atomic_test_bit(&data.state, STATE_TX_BUSY) || data.tx_cb == NULL ||
            !atomic_test_bit(&data.state, STATE_ENABLED) ||
            !atomic_test_bit(&data.state, STATE_HOST_READY)) {
        return;
    }

    uint8_t *p;
    size_t n;
    bool is_prefix = atomic_test_bit(&data.state, STATE_PREFIX) && data.prefix_len > 0;

    if (is_prefix) {
        p = prefix_buf;
        n = data.prefix_len;
    } else {
        p = (uint8_t *)cb_peek(data.tx_cb);
        n = MIN(cb_peek_avail(data.tx_cb), CONFIG_UCUSB_Z_TX_MAX_SIZE);
        if ((n & 3U) != 0U || !IS_ALIGNED(p, 4)) {
            // Cannot happen with UC_LOG_TX_ALIGN4. DMA would fail silently, so refuse.
            if (data.tx_errors++ == 0) {
                LOG_ERROR("ring run %p/%zu not word aligned", (void *)p, n);
            }
            return;
        }
    }

    if (n == 0) {
        return;
    }

    struct net_buf *buf = net_buf_alloc_with_data(&ucusb_in_pool, p, n, K_NO_WAIT);
    if (buf == NULL) {
        if (data.tx_errors++ == 0) {
            LOG_ERROR("no IN net_buf");
        }
        return;
    }

    struct udc_buf_info *bi = udc_get_buf_info(buf);
    bi->ep = bulk_in_ep(data.c_data);

    // A transfer that ends on a full packet needs a ZLP, or the host keeps waiting for more
    if ((n % (is_hs(data.c_data) ? BULK_MPS_HS : BULK_MPS_FS)) == 0) {
        udc_ep_buf_set_zlp(buf);
    }

    data.tx_len = n;
    data.tx_is_prefix = is_prefix;
    atomic_set_bit(&data.state, STATE_TX_BUSY);

    int ret = usbd_ep_enqueue(data.c_data, buf);
    if (ret != 0) {
        atomic_clear_bit(&data.state, STATE_TX_BUSY);
        net_buf_unref(buf);
        if (data.tx_errors++ == 0) {
            LOG_ERROR("IN enqueue failed %d", ret);
        }
    }
}

static void rx_arm(struct usbd_class_data *const c_data)
{
    struct net_buf *buf = net_buf_alloc(&ucusb_out_pool, K_NO_WAIT);
    if (buf == NULL) {
        LOG_ERROR("no OUT net_buf");
        return;
    }

    udc_get_buf_info(buf)->ep = bulk_out_ep(c_data);

    int ret = usbd_ep_enqueue(c_data, buf);
    if (ret != 0) {
        LOG_ERROR("OUT enqueue failed %d", ret);
        net_buf_unref(buf);
    }
}

static void host_connected(void)
{
    k_timer_start(&data.ping_timeout_timer, K_MSEC(PING_TIMEOUT_MS), K_NO_WAIT);

    if (!atomic_test_and_set_bit(&data.state, STATE_HOST_READY)) {
        atomic_set_bit(&data.state, STATE_PREFIX);
        LOG_INFO("host connected");
        tx_kick();
    }
}

static void ping_timeout(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    if (atomic_test_and_clear_bit(&data.state, STATE_HOST_READY)) {
        LOG_WARN("Ping timeout expired: Host disconnected");
    }
}

// USB stack thread
static int ucusb_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
    struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
    struct udc_buf_info *bi = udc_get_buf_info(buf);

    if (bi->ep == bulk_in_ep(c_data)) {
        if (err == 0) {
            atomic_set_bit(&data.state, STATE_TX_DONE);
        } else {
            // Not retired from the ring: the data is sent again after reconnect
            if (err != -ECONNABORTED) {
                LOG_ERROR("IN transfer failed %d", err);
            }
            atomic_clear_bit(&data.state, STATE_TX_BUSY);
        }
        usbd_ep_buf_free(uds_ctx, buf);
        tx_kick();
        return 0;
    }

    if (bi->ep == bulk_out_ep(c_data)) {
        if (err == 0 && buf->len > 0) {
            size_t n = buf->len;
            size_t avail = cb_write_avail(&data.rx_cb);
            if (n > avail) {
                LOG_ERROR("dropping rx data n:%zu", n - avail);
                n = avail;
            }
            cb_write(&data.rx_cb, buf->data, n);
            host_connected();
            k_event_post(&data.event, UCUART_EVT_RX);
        } else if (err != 0 && err != -ECONNABORTED) {
            LOG_ERROR("OUT transfer failed %d", err);
        }
        usbd_ep_buf_free(uds_ctx, buf);

        if (err != -ECONNABORTED && atomic_test_bit(&data.state, STATE_ENABLED)) {
            rx_arm(c_data);
        }
        return 0;
    }

    return usbd_ep_buf_free(uds_ctx, buf);
}

static void ucusb_update(struct usbd_class_data *const c_data, uint8_t iface, uint8_t alternate)
{
    ARG_UNUSED(c_data);
    ARG_UNUSED(iface);
    ARG_UNUSED(alternate);
}

static void ucusb_enable(struct usbd_class_data *const c_data)
{
    LOG_INFO("configured, %s speed", is_hs(c_data) ? "high" : "full");
    atomic_set_bit(&data.state, STATE_ENABLED);
    rx_arm(c_data);
    tx_kick();
}

static void ucusb_disable(struct usbd_class_data *const c_data)
{
    ARG_UNUSED(c_data);

    atomic_clear_bit(&data.state, STATE_ENABLED);
    atomic_clear_bit(&data.state, STATE_HOST_READY);
    k_timer_stop(&data.ping_timeout_timer);
    LOG_INFO("unconfigured");
}

static int ucusb_cth(struct usbd_class_data *const c_data,
        const struct usb_setup_packet *const setup, struct net_buf *const buf)
{
    ARG_UNUSED(c_data);

    if (setup->bRequest == GET_LINE_CODING) {
        if (buf == NULL) {
            errno = -ENOMEM;
            return 0;
        }
        net_buf_add_mem(buf, &data.line_coding, MIN(sizeof(data.line_coding), setup->wLength));
        return 0;
    }

    errno = -ENOTSUP;
    return 0;
}

static int ucusb_ctd(struct usbd_class_data *const c_data,
        const struct usb_setup_packet *const setup, const struct net_buf *const buf)
{
    ARG_UNUSED(c_data);

    switch (setup->bRequest) {
    case SET_LINE_CODING:
        if (setup->wLength != sizeof(data.line_coding)) {
            errno = -ENOTSUP;
            return 0;
        }
        memcpy(&data.line_coding, buf->data, sizeof(data.line_coding));
        return 0;

    case SET_CONTROL_LINE_STATE:
        // Informational only: the connection follows the ucLog ping, not DTR
        data.line_state = setup->wValue;
        return 0;

    default:
        break;
    }

    errno = -ENOTSUP;
    return 0;
}

static int ucusb_class_init(struct usbd_class_data *const c_data)
{
    ARG_UNUSED(c_data);

    ucusb_desc.if0_union.bControlInterface = ucusb_desc.if0.bInterfaceNumber;
    ucusb_desc.if0_union.bSubordinateInterface0 = ucusb_desc.if1.bInterfaceNumber;
    return 0;
}

static void *ucusb_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
    ARG_UNUSED(c_data);

#if USBD_SUPPORTS_HIGH_SPEED
    if (speed == USBD_SPEED_HS) {
        return ucusb_hs_desc;
    }
#else
    ARG_UNUSED(speed);
#endif
    return ucusb_fs_desc;
}

static const struct usbd_class_api ucusb_class_api = {
    .request = ucusb_request,
    .update = ucusb_update,
    .enable = ucusb_enable,
    .disable = ucusb_disable,
    .control_to_host = ucusb_cth,
    .control_to_dev = ucusb_ctd,
    .init = ucusb_class_init,
    .get_desc = ucusb_get_desc,
};

USBD_DEFINE_CLASS(ucusb_z_0, &ucusb_class_api, (void *)DEVICE_DT_INST_GET(0), NULL);

// ucuart API

static int tx_schedule(const struct device *dev)
{
    ARG_UNUSED(dev);

    if (data.tx_cb == NULL || !atomic_test_bit(&data.state, STATE_HOST_READY)) {
        return -ENOTCONN;
    }
    tx_kick();
    return 0;
}

static int set_connect_prefix(const struct device *dev, const uint8_t *prefix, size_t pn)
{
    ARG_UNUSED(dev);

    if (prefix == NULL || pn == 0) {
        data.prefix_len = 0;
        return 0;
    }

    size_t padded = ROUND_UP(pn, 4);
    if (padded > sizeof(prefix_buf)) {
        LOG_ERROR("Prefix too long: %zu", pn);
        data.prefix_len = 0;
        return -ENOMEM;
    }

    memcpy(prefix_buf, prefix, pn);
    memset(prefix_buf + pn, 0, padded - pn);
    data.prefix_len = padded;
    return 0;
}

// Direct writes are not padded, so this driver refuses them. log.c only uses the ring.
static int tx(const struct device *dev, const uint8_t *b, size_t n)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(b);
    ARG_UNUSED(n);
    return -ENOTSUP;
}

static int tx_buffer(const struct device *dev, const uint8_t *b, size_t n)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(b);
    ARG_UNUSED(n);
    return -ENOTSUP;
}

static int set_tx_cb(const struct device *dev, cb_t *cb)
{
    ARG_UNUSED(dev);

    data.tx_cb = cb;
    tx_kick();
    return 0;
}

static void rx_start(const struct device *dev)
{
    ARG_UNUSED(dev);
}

static void rx_stop(const struct device *dev)
{
    ARG_UNUSED(dev);
}

static size_t rx_avail(const struct device *dev)
{
    ARG_UNUSED(dev);
    return cb_peek_avail(&data.rx_cb);
}

static const uint8_t *rx_peek(const struct device *dev)
{
    ARG_UNUSED(dev);
    return cb_peek(&data.rx_cb);
}

static void rx_skip(const struct device *dev, size_t n)
{
    ARG_UNUSED(dev);
    cb_skip(&data.rx_cb, n);
}

static uint32_t wait_event(const struct device *dev, uint32_t mask, bool reset, k_timeout_t timeout)
{
    // Events between k_event_wait and k_event_clear are fine: the client drains rx_avail() before
    // waiting again.
    ARG_UNUSED(dev);
    ARG_UNUSED(reset);

    uint32_t r = k_event_wait(&data.event, mask, false, timeout);
    if (r != 0) {
        k_event_clear(&data.event, r);
    }
    return r;
}

static int panic(const struct device *dev)
{
    ARG_UNUSED(dev);
    // TODO: flushing from a fault needs the USB interrupt polled with interrupts disabled
    return 0;
}

static int is_host_ready(const struct device *dev, bool *ready)
{
    ARG_UNUSED(dev);
    *ready = atomic_test_bit(&data.state, STATE_HOST_READY);
    return 0;
}

static const struct ucuart_driver_api ucusb_api = {
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
    .is_host_ready = is_host_ready,
};

// USB device

USBD_DEVICE_DEFINE(ucusb_usbd, DEVICE_DT_GET(DT_INST_BUS(0)), CONFIG_UCUSB_Z_VID,
        CONFIG_UCUSB_Z_PID);

USBD_DESC_LANG_DEFINE(ucusb_lang);
USBD_DESC_MANUFACTURER_DEFINE(ucusb_mfr, CONFIG_UCUSB_Z_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(ucusb_product, CONFIG_UCUSB_Z_PRODUCT);
USBD_DESC_SERIAL_NUMBER_DEFINE(ucusb_sn);

USBD_DESC_CONFIG_DEFINE(ucusb_fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(ucusb_hs_cfg_desc, "HS Configuration");

// Powered by the instrument, not the USB port
USBD_CONFIGURATION_DEFINE(ucusb_fs_config, USB_SCD_SELF_POWERED, 0, &ucusb_fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(ucusb_hs_config, USB_SCD_SELF_POWERED, 0, &ucusb_hs_cfg_desc);

static void usbd_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
    ARG_UNUSED(ctx);

    if (msg->type == USBD_MSG_UDC_ERROR || msg->type == USBD_MSG_STACK_ERROR) {
        LOG_ERROR("usbd: %s %d", usbd_msg_type_string(msg->type), msg->status);
    } else {
        LOG_INFO("usbd: %s", usbd_msg_type_string(msg->type));
    }
}

static int add_configuration(struct usbd_context *ctx, enum usbd_speed speed,
        struct usbd_config_node *config)
{
    int err = usbd_add_configuration(ctx, speed, config);
    if (err != 0) {
        LOG_ERROR("add configuration: %d", err);
        return err;
    }

    err = usbd_register_class(ctx, ucusb_z_0.name, speed, 1);
    if (err != 0) {
        LOG_ERROR("register class: %d", err);
        return err;
    }

    // IAD: Miscellaneous device class, common class, IAD protocol
    return usbd_device_set_code_triple(ctx, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
}

static int ucusb_usb_start(void)
{
    struct usbd_context *ctx = &ucusb_usbd;
    int err;

    err = usbd_add_descriptor(ctx, &ucusb_lang);
    err = err ? err : usbd_add_descriptor(ctx, &ucusb_mfr);
    err = err ? err : usbd_add_descriptor(ctx, &ucusb_product);
    err = err ? err : usbd_add_descriptor(ctx, &ucusb_sn);
    if (err != 0) {
        LOG_ERROR("add descriptors: %d", err);
        return err;
    }

    if (USBD_SUPPORTS_HIGH_SPEED && usbd_caps_speed(ctx) == USBD_SPEED_HS) {
        err = add_configuration(ctx, USBD_SPEED_HS, &ucusb_hs_config);
        if (err != 0) {
            return err;
        }
    }

    err = add_configuration(ctx, USBD_SPEED_FS, &ucusb_fs_config);
    if (err != 0) {
        return err;
    }

    usbd_self_powered(ctx, true);

    err = usbd_msg_register_cb(ctx, usbd_msg_cb);
    if (err != 0) {
        LOG_ERROR("register msg cb: %d", err);
        return err;
    }

    err = usbd_init(ctx);
    if (err != 0) {
        LOG_ERROR("usbd_init: %d", err);
        return err;
    }

    err = usbd_enable(ctx);
    if (err != 0) {
        LOG_ERROR("usbd_enable: %d", err);
        return err;
    }

    return 0;
}

static int ucusb_init(const struct device *dev)
{
    data.dev = dev;
    data.c_data = &ucusb_z_0;

    cb_init(&data.rx_cb, data.rx_buf, sizeof(data.rx_buf));
    k_event_init(&data.event);
    k_timer_init(&data.ping_timeout_timer, ping_timeout, NULL);
    k_work_init(&data.tx_work, tx_work_handler);

    k_work_queue_init(&data.work_q);
    k_work_queue_start(&data.work_q, data.stack_area, K_THREAD_STACK_SIZEOF(data.stack_area),
            CONFIG_UCUSB_Z_TX_WQ_THREAD_PRIORITY, NULL);
    k_thread_name_set(&data.work_q.thread, "UCUSB TX WQ");

    return ucusb_usb_start();
}

DEVICE_DT_INST_DEFINE(0, ucusb_init, NULL, &data, NULL, POST_KERNEL, CONFIG_UCUSB_Z_INIT_PRIORITY,
        &ucusb_api);

// © 2022 Unit Circle Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <zephyr/device.h>
#include <zephyr/toolchain.h>
#include <zephyr/kernel.h>
#include "cb.h"

#define ZEPHYR_DEVICE_MEMBER(dev, member) ((dev)->member)

#ifdef __cplusplus
extern "C" {
#endif

// @cond INTERNAL_HIDDEN

typedef int (*ucuart_tx_no_wait_t)(const struct device *dev, const uint8_t* b, size_t n);
typedef int (*ucuart_tx_buffer_t)(const struct device *dev, const uint8_t* b, size_t n);
typedef int (*ucuart_tx_schedule_t)(const struct device *dev);
typedef int (*ucuart_set_connect_prefix_t)(const struct device *dev, const uint8_t* prefix, size_t pn);
typedef int (*ucuart_set_tx_cb_t)(const struct device *dev, cb_t* cb);


typedef void (*ucuart_rx_start_t)(const struct device *dev);
typedef void (*ucuart_rx_stop_t)(const struct device *dev);
typedef size_t (*ucuart_rx_avail_t)(const struct device *dev);
typedef const uint8_t* (*ucuart_rx_peek_t)(const struct device *dev);
typedef void (*ucuart_rx_skip_t)(const struct device *dev, size_t n);

typedef uint32_t (*ucuart_wait_event_t)(const struct device *dev, uint32_t mask, bool reset, k_timeout_t timeout);

typedef int (*ucuart_panic_t)(const struct device *dev);

typedef int (*ucuart_is_host_ready_t)(const struct device *dev, bool *ready);

enum ucuart_event_e {
  UCUART_EVT_RX = 1,
};

typedef const struct device uart_t;


// Create a ucuart as new api that uses the uarte wiht DMA.
// Unfortunatley pincntl has conditional compiles for UART specific pin
// configuration - so zephyr/drivers/pinctrl/pinctrl_nrf.c has to be patched.

__subsystem struct ucuart_driver_api {
  ucuart_tx_no_wait_t tx_no_wait;
  ucuart_tx_buffer_t tx_buffer;
  ucuart_tx_schedule_t tx_schedule;
  ucuart_set_connect_prefix_t set_connect_prefix;
  ucuart_set_tx_cb_t set_tx_cb;

  ucuart_rx_start_t rx_start;
  ucuart_rx_stop_t rx_stop;
  ucuart_rx_avail_t rx_avail;
  ucuart_rx_peek_t rx_peek;
  ucuart_rx_skip_t rx_skip;

  ucuart_wait_event_t wait_event;

  ucuart_panic_t panic;

  ucuart_is_host_ready_t is_host_ready;
};

// @endcond


/**
 * @brief Send any buffered data.
 *
 * @param dev UcUart device instance.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int ucuart_tx_schedule(const struct device *dev);

static inline int z_impl_ucuart_tx_schedule(const struct device *dev) {
  const struct ucuart_driver_api *api =
      (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);

  return api->tx_schedule(dev);
}

/**
 * @brief Set prefix to send first on new connection.
 *
 * @param dev UcUart device instance.
 * @param prefix Buffer of bytes to send before sending data in the circ buffer
 * @param pn Length of prefix buffer.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int ucuart_set_connect_prefix(const struct device *dev, const uint8_t* prefix, size_t pn);

static inline int z_impl_ucuart_set_connect_prefix(const struct device *dev, const uint8_t* prefix, size_t pn) {
  const struct ucuart_driver_api *api =
      (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);

  return api->set_connect_prefix(dev, prefix, pn);
}

/**
 * @brief Send data.
 *
 * @param dev UcUart device instance.
 * @param b Buffer of bytes to send.
 * @param n Length of buffer.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int ucuart_tx_no_wait(const struct device *dev, const uint8_t* b, size_t n);

static inline int z_impl_ucuart_tx_no_wait(const struct device *dev, const uint8_t* b, size_t n) {
  const struct ucuart_driver_api *api =
    (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);

  return api->tx_no_wait(dev, b, n);
}

/**
 * @brief Buffer data to send - but don't send.
 *
 * @param dev UcUart device instance.
 * @param b Buffer of bytes to buffer.
 * @param n Length of buffer.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int ucuart_tx_buffer(const struct device *dev, const uint8_t* b, size_t n);

static inline int z_impl_ucuart_tx_buffer(const struct device *dev, const uint8_t* b, size_t n)
{
        const struct ucuart_driver_api *api =
                (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);

        return api->tx_buffer(dev, b, n);
}

/**
 * @brief Set tx circular buffer - must be called before any of the other apis
 *
 * @param dev UcUart device instance.
 * @param cb  circular buffer instance.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int ucuart_set_tx_cb(const struct device *dev, cb_t* cb);

static inline int z_impl_ucuart_set_tx_cb(const struct device *dev, cb_t* cb)
{
        const struct ucuart_driver_api *api =
                (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);

        return api->set_tx_cb(dev, cb);
}

__syscall void ucuart_rx_start(const struct device *dev);

static inline void z_impl_ucuart_rx_start(const struct device *dev) {
  const struct ucuart_driver_api *api =
    (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);
  api->rx_start(dev);
}

__syscall void ucuart_rx_stop(const struct device *dev);

static inline void z_impl_ucuart_rx_stop(const struct device *dev) {
  const struct ucuart_driver_api *api =
    (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);
  api->rx_stop(dev);
}

__syscall size_t ucuart_rx_avail(const struct device *dev);

static inline size_t z_impl_ucuart_rx_avail(const struct device *dev) {
  const struct ucuart_driver_api *api =
    (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);
  return api->rx_avail(dev);
}

__syscall const uint8_t* ucuart_rx_peek(const struct device *dev);

static inline const uint8_t* z_impl_ucuart_rx_peek(const struct device *dev) {
  const struct ucuart_driver_api *api =
    (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);
  return api->rx_peek(dev);
}

__syscall void ucuart_rx_skip(const struct device *dev, size_t n);

static inline void z_impl_ucuart_rx_skip(const struct device *dev, size_t n) {
  const struct ucuart_driver_api *api =
    (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);
  api->rx_skip(dev, n);
}

__syscall uint32_t ucuart_wait_event(const struct device *dev, uint32_t mask, bool reset, k_timeout_t timeout);

static inline uint32_t z_impl_ucuart_wait_event(const struct device *dev, uint32_t mask, bool reset, k_timeout_t timeout) {
  const struct ucuart_driver_api *api =
    (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);
  return api->wait_event(dev, mask, reset, timeout);
}


/**
 * @brief Switch to panic mode - assumes interrupts disabled
 *
 * @param dev UcUart device instance.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int ucuart_panic(const struct device *dev);

static inline int z_impl_ucuart_panic(const struct device *dev) {
  const struct ucuart_driver_api *api =
      (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);

  return api->panic(dev);
}

__syscall int ucuart_is_host_ready(const struct device *dev, bool *ready);

static inline int z_impl_ucuart_is_host_ready(const struct device *dev,
                                              bool *ready) {
  const struct ucuart_driver_api *api =
    (const struct ucuart_driver_api *)ZEPHYR_DEVICE_MEMBER(dev, api);
  if (api->is_host_ready == NULL) {
    return -ENOSYS;
  }
  return api->is_host_ready(dev, ready);
}

#include <syscalls/ucuart.h>

#ifdef __cplusplus
}
#endif

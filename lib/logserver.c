// © 2025 Unit Circle Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include "log.h"
#include "cobs.h"
#include "cb.h"

#if defined(CONFIG_LOG_CUSTOM_HEADER)
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#endif

// Allow others to implement a watchdog timer if needed
__weak void log_watchdog_feed(void) {
}

__weak void log_watchdog_register(const void* thread) {
  (void) thread;
}

#if !defined(CONFIG_UC_LOG_SERVER_PORTS)
#define CONFIG_UC_LOG_SERVER_PORTS (8)
#endif


typedef struct {
  const struct device* uart;
  uint8_t buf[COBS_ENC_SIZE(LOG_MAX_PACKET_SIZE)+3];
  cb_t    cb;
  bool    overrun;
  uint8_t* hash[LOG_APP_HASH_SIZE + 2];
#if CONFIG_UC_LOG_SERVER_PORTS> 0
  log_cb_t* handlers[CONFIG_UC_LOG_SERVER_PORTS];
  void*     contexts[CONFIG_UC_LOG_SERVER_PORTS];
  log_cb_t* shell_handler;
  void*     shell_context;
  uint8_t port;
  uint8_t  rx_port;
  bool     rx_avail;
  uint8_t* rx_data;
  size_t   rx_n;
#endif
#if defined(CONFIG_LOG_CUSTOM_HEADER)
  struct k_event rx_event;
  K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_UC_LOG_STACK_SIZE);
  struct k_thread thread;
#endif
} log_server_data_t;

static log_server_data_t server;

size_t log_rx(uint8_t port, uint8_t* data, size_t n) {
  if (port >= LOG_MAX_PORTS) LOG_FATAL("Invalid port %d", port);
  if (server.rx_port != 255) LOG_FATAL("Trying to call log_rx from another thread");
  server.rx_data = data;
  server.rx_n = n;
  server.rx_port = port;

  while (server.rx_port != 255) {
#if defined(CONFIG_LOG_CUSTOM_HEADER)
    unsigned int key = irq_lock();
    uint32_t r =  k_event_wait(&server.rx_event, 1, false, K_MSEC(1000));
    if (r != 0) k_event_clear(&server.rx_event, r);
    irq_unlock(key);
#else
    // Non-zephyr "wait for event"
#endif

    log_watchdog_feed();
  }

  server.rx_port = 255;
  return server.rx_n;
}

void log_notify(uint8_t port, log_cb_t* task, void* ctx) {
  if (port >= CONFIG_UC_LOG_SERVER_PORTS) {
    LOG_FATAL("port out of range: %d", port);
  }
  server.handlers[port] = task;
  server.contexts[port] = ctx;
}

void log_notify_shell(log_cb_t* task, void* ctx) {
  server.shell_handler = task;
  server.shell_context = ctx;
}

static void log_thread(log_server_data_t* data) {
  LOG_INFO("log thread starting");

pause:
  log_tx_suspend();
  //LOG_INFO("log pause");
  ucuart_rx_stop(data->uart);
  size_t n = ucuart_rx_avail(data->uart);
  ucuart_rx_skip(data->uart, n);
  n = ucuart_rx_avail(data->uart);
  ucuart_rx_skip(data->uart, n);
  n = ucuart_rx_avail(data->uart);

  while (n == 0u) {
#if defined(CONFIG_LOG_CUSTOM_HEADER)
    uint32_t r = ucuart_wait_event(data->uart, UCUART_EVT_RX, false, K_MSEC(1000));
#else
        // Non-zephyr way to wait for event
#endif
    log_watchdog_feed();
    if (r != 0) break;
    n = ucuart_rx_avail(data->uart);
  }
  ucuart_rx_start(data->uart);
  //LOG_INFO("log resume");
  log_tx_resume();

  while (true) {
    // Wait for a start of frame
    while (true) {
      size_t n = ucuart_rx_avail(data->uart);

      // It is possible to have a pending UCUART_EVT_RX even though previous
      // handling has drained the queued rx data.
      // E.g.
      // log process wait for UCUART_EVT_RX
      //   post UCUART_EVT_RX avail: 64 (setting event)
      // log process starts running (clearing event)
      // log process wait for UCUART_EVT_RX
      //   post UCUART_EVT_RX avail: 64 (setting event)
      // log process starts running (clearing event)
      //   post UCUART_EVT_RX avail: 3 (setting event)
      // log process continues running - draining queued data
      // log process wait for UCUART_EVT_RX
      // log process starts running (clearing event)
      // log process finds there is no new data.
      while (n == 0u) {
#if defined(CONFIG_LOG_CUSTOM_HEADER)
        uint32_t r = ucuart_wait_event(data->uart, UCUART_EVT_RX, false, K_MSEC(1000));
#else
        // Non-zephyr way to wait for event
#endif

        log_watchdog_feed();

        if (r == 0) goto pause;
        n = ucuart_rx_avail(data->uart);
      }

      const uint8_t*b = ucuart_rx_peek(data->uart);
      if (*b != '\0') break;
      ucuart_rx_skip(data->uart, 1);
    }

    // Process until end of frame
    cb_reset(&data->cb);
    data->overrun = false;
    while (true) {
      size_t n = ucuart_rx_avail(data->uart);

      while (n == 0u) {
#if defined(CONFIG_LOG_CUSTOM_HEADER)
        uint32_t r = ucuart_wait_event(data->uart, UCUART_EVT_RX, false, K_MSEC(100));
#else
        // Non-zephyr way to wait for event
#endif
        log_watchdog_feed();
        if (r == 0) goto pause;
        n = ucuart_rx_avail(data->uart);
      }

      const uint8_t* b = ucuart_rx_peek(data->uart);
      const uint8_t* e = memchr(b, '\0', n);
      if (e != NULL) {
        n = e - b;
        if (n > cb_write_avail(&data->cb)) {
          data->overrun = true;
          n = cb_write_avail(&data->cb);
        }
        cb_write(&data->cb, b, n);
        //LOG_MEM_INFO("Rx: ", data->buf,  cb_peek_avail(&data->cb));
        ucuart_rx_skip(data->uart, e-b); // Leave the 0x00 frame terminator
        ssize_t n = cobs_dec(data->buf, data->buf, cb_peek_avail(&data->cb));
        if ((n < 0) || data->overrun) {
          LOG_ERROR("COBS decode error: %d overrun: %d", (int) n, data->overrun);
        }
        else if (n > 0) {
          uint8_t type = data->buf[0] & 3;
          data->port = data->buf[0] >> 2;
          cb_skip(&data->cb, 1);
          if (type != 0x3) {
            LOG_ERROR("unexpected frame type: %d", type);
          }
          else if ((data->rx_port < LOG_MAX_PORTS) && (data->port == data->rx_port)) {
            size_t nn = n - 1;
            if (nn > data->rx_n) {
              nn = data->rx_n;
              LOG_WARN("rx_port buffer size too small");
            }
            memmove(data->rx_data, data->buf + 1, nn);
            data->rx_n = n - 1;
            data->rx_port = 255;
#if defined(CONFIG_LOG_CUSTOM_HEADER)
            k_event_post(&data->rx_event, 1);
#else
            // Non-zephyr way to post an event
#endif
          }
          else if (data->port == LOG_PORT_SHELL && data->shell_handler) {
            data->shell_handler(data->buf + 1, n - 1, data->shell_context);
          }
          else if (data->port >= CONFIG_UC_LOG_SERVER_PORTS) {
            LOG_ERROR("invalid port: %d", data->port);
          }
          else if (data->handlers[data->port]) {
            data->handlers[data->port](data->buf + 1, n - 1,
                data->contexts[data->port]);
          }
          else {
            LOG_ERROR("no handler for port: %d", data->port);
            LOG_MEM_ERROR("data:", data->buf+1, n-1);
          }
        }
        else {
          LOG_INFO("empty frame");
          // Ignore empty frames
        }
        break;
      }
      else {
        size_t n2 = n;
        if (n > cb_write_avail(&data->cb)) {
          data->overrun = true;
          n = cb_write_avail(&data->cb);
        }
        cb_write(&data->cb, b, n);
        ucuart_rx_skip(data->uart, n2);
      }
    }
  }
}

void log_server_init(uart_t* uart) {
  server.uart = uart;
  server.rx_port = 255;

  cb_init(&server.cb, server.buf, sizeof(server.buf));
#if CONFIG_UC_LOG_SERVER_PORTS > 0
  memset(server.handlers, 0, sizeof(server.handlers));
#endif
}

static const struct device* console = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_console));

#if defined(CONFIG_LOG_CUSTOM_HEADER)
static int z_log_server_init(void) {
  if (!device_is_ready(console)) return -ENOTSUP;
  log_server_init(console);

#if CONFIG_UC_LOG_SERVER_PORTS > 0
  k_event_init(&server.rx_event);

  k_tid_t tid = k_thread_create(&server.thread, server.thread_stack,
                        CONFIG_UC_LOG_STACK_SIZE,
                        (k_thread_entry_t)log_thread,
                        &server, NULL, NULL,
                        K_PRIO_COOP(CONFIG_UC_LOG_THREAD_PRIORITY),
                        0, K_NO_WAIT);
  if (k_thread_name_set(tid, "Log") != 0) {
      // Couldn't set thread name
  }

  log_watchdog_register(tid);
#endif
  return 0;
}
SYS_INIT(z_log_server_init, APPLICATION, 20);
#endif

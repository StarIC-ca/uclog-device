// © 2025 Unit Circle Inc.
// SPDX-License-Identifier: Apache-2.0

#include <zephyr/shell/shell.h>
#include <zephyr/init.h>
#include "cb.h"
#include "log.h"

// Specific notify for shell, not in public log.h
extern void log_notify_shell(log_cb_t* task, void* ctx);

struct shell_uc_ctrl_blk {
  shell_transport_handler_t handler;
  void *context;
};

struct shell_uc {
  struct shell_uc_ctrl_blk *ctrl_blk;
  cb_t *rx_cb;
  cb_t *tx_cb;
  struct k_mutex mutex;
};

extern const struct shell_transport_api transport_api;

#define SHELL_UC_DEFINE(_name, _rx_cb_size)  \
  static struct shell_uc_ctrl_blk _name##_ctrl_blk;    \
  static uint8_t _name##_rx_buf[_rx_cb_size]; \
  static uint8_t _name##_tx_buf[200]; \
  static cb_t    _name##_rx_cb = CB_INIT(_name##_rx_buf); \
  static cb_t    _name##_tx_cb = CB_INIT(_name##_tx_buf); \
  static struct shell_uc _name##_shell_uc = {    \
    .ctrl_blk = &_name##_ctrl_blk,        \
    .rx_cb = &_name##_rx_cb,      \
    .tx_cb = &_name##_tx_cb,      \
  };                \
  struct shell_transport _name = {        \
    .api = &transport_api,      \
    .ctx = (struct shell_uc *)&_name##_shell_uc    \
  }


#define CONFIG_UC_SHELL_BACKEND_SERIAL_RX_BUF_SIZE (256)
#define CONFIG_UC_SHELL_BACKEND_INIT_PRIORITY 10

SHELL_UC_DEFINE(shell_transport_uc,
      CONFIG_UC_SHELL_BACKEND_SERIAL_RX_BUF_SIZE);

SHELL_DEFINE(shell_uc, CONFIG_UC_SHELL_PROMPT, &shell_transport_uc,
       0,
       0,
       SHELL_FLAG_OLF_CRLF);

static void log_handle(const uint8_t* rx, size_t rx_n, void* ctx) {
  const struct shell_uc *sh = (struct shell_uc *)ctx;
  size_t wa = cb_write_avail(sh->rx_cb);
  if (wa < rx_n) {
    LOG_ERROR("log_handle overflow wa: %zu rx_n: %zu", wa, rx_n);
    rx_n = wa;
  }
  if (rx_n > 0) {
    cb_write(sh->rx_cb, rx, rx_n);
    sh->ctrl_blk->handler(SHELL_TRANSPORT_EVT_RX_RDY,
             sh->ctrl_blk->context);
  }
}

static int init(const struct shell_transport *transport,
    const void *config,
    shell_transport_handler_t evt_handler,
    void *context) {
  struct shell_uc *sh = (struct shell_uc *)transport->ctx;

  (void) config;
  k_mutex_init(&sh->mutex);
  sh->ctrl_blk->handler = evt_handler;
  sh->ctrl_blk->context = context;

  log_notify_shell(log_handle, (void*) sh);
  return 0;
}

static int uninit(const struct shell_transport *transport) {
  (void) transport;
  log_notify_shell(NULL, NULL);
  return 0;
}

static int enable(const struct shell_transport *transport, bool blocking_tx) {
  (void) transport;
  (void) blocking_tx;
  return 0;
}

static int write(const struct shell_transport *transport,
     const void *data, size_t length, size_t *cnt) {
  struct shell_uc *sh = (struct shell_uc *)transport->ctx;
  *cnt = length;

  // Need mutex to ensure that only one thread at a time can update
  // queued data and flush at a time.  Normally there is only one caller to
  // shell_print at time - but we enforce that here.
  if (k_mutex_lock(&sh->mutex, K_MSEC(100)) != 0) {
    LOG_ERROR("unabled to obtain lock");
    return 0;
  }
  // Flush until enough space
  while (length > cb_write_avail(sh->tx_cb)) {
    size_t n = cb_peek_avail(sh->tx_cb);
    if (n == 0) break;
    //LOG_MEM_INFO("sh:", cb_peek(sh->tx_cb), n);
    log_tx(LOG_PORT_SHELL, cb_peek(sh->tx_cb), n);
    cb_skip(sh->tx_cb, n);
  }
  cb_write(sh->tx_cb, data, length);

  // If we have EOL or Prompt then flush right away
  const uint8_t* buf = data;
  if ((length >= 2) &&
         (((buf[length-2] == 0x0d) && (buf[length-1] == 0x0a)) || // EOL
          ((buf[length-2] == '~') && (buf[length-1] == ' ')))) {   // prompt
    // Flush
    while (true) {
      size_t n = cb_peek_avail(sh->tx_cb);
      if (n == 0) break;
      //LOG_MEM_INFO("sh:", cb_peek(sh->tx_cb), n);
      log_tx(LOG_PORT_SHELL, cb_peek(sh->tx_cb), n);
      cb_skip(sh->tx_cb, n);
    }
  }
  k_mutex_unlock(&sh->mutex);
  // NOTE: if return *cnt == 0 then need to return SHELL_TRANSPORT_EVT_TX_RDY
  //sh->ctrl_blk->handler(SHELL_TRANSPORT_EVT_TX_RDY,
  //           sh->ctrl_blk->context);
  return 0;
}

static int read(const struct shell_transport *transport,
    void *data, size_t length, size_t *cnt) {
  struct shell_uc *sh = (struct shell_uc *)transport->ctx;

  *cnt = cb_read_avail(sh->rx_cb);
  if (*cnt > length) *cnt = length;
  cb_read(sh->rx_cb, data, *cnt);

  return 0;
}

const struct shell_transport_api transport_api = {
  .init = init,
  .uninit = uninit,
  .enable = enable,
  .write = write,
  .read = read,
};

static int enable_shell(void) {
  static const struct shell_backend_config_flags cfg_flags =
          SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;

  shell_init(&shell_uc, NULL, cfg_flags, false, 0);

  return 0;
}

SYS_INIT(enable_shell, APPLICATION, CONFIG_UC_SHELL_BACKEND_INIT_PRIORITY);

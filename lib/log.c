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
#include "cbor.h"
#include "cb.h"

#ifdef CONFIG_APP_VERSION
#include "app_version.h"
#endif

#if CONFIG_UC_LOG_SAVE
#define NOCLEAR __noinit
#else
#define NOCLEAR
#endif

#if !defined(CONFIG_UC_LOG_MAX_PACKET_SIZE)
#define CONFIG_UC_LOG_MAX_PACKET_SIZE (1500)
#endif

typedef struct {
  const uart_t* uart;
  bool    tx_enabled;
} log_data_t;

static log_data_t log_data;

#if !defined(CONFIG_UC_LOG_BUF_SIZE)
#define CONFIG_UC_LOG_BUF_SIZE (4096*2)
#endif

#if defined(CONFIG_UCUART_Z) && defined(CONFIG_SOC_FAMILY_STM32) && defined(CONFIG_NOCACHE_MEMORY)
#define NOCACHE_ATTR __nocache
#else
#define NOCACHE_ATTR
#endif

static NOCLEAR cb_t    tx_cb;
static NOCLEAR uint8_t tx_buf[CONFIG_UC_LOG_BUF_SIZE] NOCACHE_ATTR;

#define MAX_DEVICE_INFO_SIZE 256
static uint8_t device_info_tx_buf[COBS_ENC_SIZE(MAX_DEVICE_INFO_SIZE) + 2] NOCACHE_ATTR;
static size_t device_info_len = 0;

static size_t strnlen_s (const char* s, size_t n) {
  const char* found = memchr(s, '\0', n);
  return found ? (size_t)(found-s) : n;
}

void log_panic_(void) {
  if (log_data.uart != NULL) {
    ucuart_panic(log_data.uart);
  }
}

static void tx_buffer(const uint8_t* b, size_t n);

void log_log1_(const char *prefix) {
  union {
    const void* p;
    uint8_t v[sizeof(const void*)];
  } v;
  uint8_t b[5+2];

  v.p = prefix;
  v.v[0] = (v.v[0] & 0xfc) | 0x00;
  memmove(b+2, v.v, 4);
  size_t n = cobs_enc(b+1, b+2, 4); // inplace
  b[0] = 0x00;
  b[1+n] = 0x00;
  tx_buffer(b, n+2);
  if (log_data.tx_enabled) ucuart_tx_schedule(log_data.uart, NULL, 0);
}

void log_logn_(const char* fmt, const char *prefix,  ...) {
  union {
    unsigned int u;
    unsigned long long int ull;
    double d;
    long double ld;
    const void* p;
    uint8_t v[16];
  } v;

  uint8_t b[100]; // Limits total packet size - code below expect less than 253
  size_t n = sizeof(b)-1-2;
  uint8_t* bb = b+1+1;
  size_t sn;

  v.p = prefix;
  v.v[0] = (v.v[0] & 0xfc) | 0x00;
  memmove(bb, v.v, 4);
  bb += 4; n -= 4;

  va_list args;
  va_start(args, prefix);
  while (*fmt != '\0') {
    switch (*fmt++) {
      case '0':
        if (n < 4) goto done;
        v.u = va_arg(args, unsigned int);
        memmove(bb, v.v, 4);
        bb += 4; n -= 4;
        break;
      case '1':
        if (n < 8) goto done;
        v.ull = va_arg(args, unsigned long long int);
        memmove(bb, v.v, 8);
        bb += 8; n -= 8;
        break;
      case '2':
        if (n < 8) goto done;
        v.d = va_arg(args, double);
        memmove(bb, v.v, 8);
        bb += 8; n -= 8;
        break;
      case '3':
        if (n < 16) goto done;
        v.ld = va_arg(args, long double);
        memmove(bb, v.v, 16);
        bb += 16; n -= 16;
        break;
      case '4':
        if (n < 1) goto done;
        v.p = va_arg(args, char*);
        sn = strnlen_s(v.p, n-1);
        memmove(bb, v.p, sn);
        bb += sn;
        *bb++ = '\0';
        n -= sn + 1;
        break;
      case '5':
        if (n < 4) goto done;
        v.p = va_arg(args, void*);
        memmove(bb, v.v, 4);
        bb += 4; n -= 4;
        break;
      default:
        goto done;
    }
  }
done:
  n = cobs_enc(b+1, b+1+1, sizeof(b) -2 - 1 - n); // inplace
  b[0] = 0x00;
  b[1+n] = 0x00;
  tx_buffer(b, n+2);
  if (log_data.tx_enabled) ucuart_tx_schedule(log_data.uart, NULL, 0);
}

void log_mem_(const char *prefix, const void* b, size_t n) {
  union {
    const void* p;
    uint8_t v[sizeof(const void*)];
  } v;
  uint8_t bb[100];

  if (n + 8 + 1 + 2> 100) n = 100 - 8 - 1 - 2;

  v.p = prefix;
  v.v[0] = (v.v[0] & 0xfc) | 0x01;
  memmove(bb+2, v.v, 4);
  v.p = b;
  memmove(bb+2+4, v.v, 4);
  memmove(bb+2+8, b, n);
  n = cobs_enc(bb+1, bb+2, 8+n); // inplace
  bb[0] = 0x00;
  bb[1+n] = 0x00;
  tx_buffer(bb, n+2);
  if (log_data.tx_enabled) ucuart_tx_schedule(log_data.uart, NULL, 0);
}

void log_tx_suspend(void) {
  log_data.tx_enabled = false;
}

#ifdef CONFIG_APP_VERSION
static void fill_device_info(void)
{
    size_t port_offset = sizeof(device_info_tx_buf) - (size_t)MAX_DEVICE_INFO_SIZE;

    // Encode port number
    uint8_t port = LOG_PORT_DEVICE_INFO;
    device_info_tx_buf[port_offset] = (port << 2) | 3;

    // Encode CBOR
    size_t cbor_offset = port_offset + 1;
    uint8_t *cbor_output = &device_info_tx_buf[cbor_offset];

    cbor_stream_t cbor_stream;
    cbor_init(&cbor_stream, cbor_output, MAX_DEVICE_INFO_SIZE);

#ifdef CONFIG_SIGNED_IMAGE
    const uint8_t *app_hash__ = sbl_app_hash();
#endif

    cbor_error_t err = cbor_pack(&cbor_stream,
            "{"
            ".app_hash:b,"
            ".image_type:s,"
            ".board:s"
            "}",
            log_app_hash(NULL), LOG_APP_HASH_SIZE,
            get_image_type(),
            CONFIG_BOARD);
    if (err != CBOR_ERROR_NONE) {
        LOG_FATAL("CBOR pack error: {enum:cbor_error_t}%d", err);
    }

    size_t cbor_len = cbor_read_avail(&cbor_stream);

    // Encode COBS
    size_t cobs_len = cobs_enc(&device_info_tx_buf[1], &device_info_tx_buf[port_offset],
            cbor_len + 1);

    // Frame
    device_info_tx_buf[0] = '\0';
    device_info_tx_buf[cobs_len + 1] = '\0';

    device_info_len = cobs_len + 2;
}
#endif

void log_tx_resume(void) {
  if (log_data.uart != NULL) {
    log_data.tx_enabled = true;
    ucuart_tx_schedule(log_data.uart, device_info_tx_buf, device_info_len);
  } else {
    LOG_ERROR("UART device not set!");
  }
}

int log_is_host_ready(bool *host_ready) {
  if (log_data.uart != NULL) {
    return ucuart_is_host_ready(log_data.uart, host_ready);
  } else {
    *host_ready = false;
    return -ENODEV;
  }
}

// Allow others to override log_fatal if needed
__weak void log_fatal(void) {
}

__attribute__((noreturn)) void log_fatal_(void) {
  log_fatal();

  // We switched to log_panic so all data will have been flushed
  // If debugger connected then bkpt, otherwise reset
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0) {
    __BKPT(0);
  }
  NVIC_SystemReset();
}

static void tx_buffer(const uint8_t* b, size_t n) {
  uint32_t key = irq_lock();
  cb_write(&tx_cb, b, n);
  irq_unlock(key);
}

void log_tx(uint8_t port, const uint8_t* data, size_t n) {
  static uint8_t b[COBS_ENC_SIZE(LOG_MAX_PACKET_SIZE+1)+2];

  if (n > LOG_MAX_PACKET_SIZE) LOG_FATAL("tx message too long %zu", n);
  if (port >= LOG_MAX_PORTS) LOG_FATAL("invalid port %d", port);
  b[sizeof(b)-(LOG_MAX_PACKET_SIZE+1)] = (port << 2) | 3;
  memmove(b+sizeof(b)-LOG_MAX_PACKET_SIZE, data, n);
  n = cobs_enc(b+1, b+sizeof(b)-(LOG_MAX_PACKET_SIZE+1), n + 1);
  b[0] = '\0';
  b[n+1] = '\0';
  tx_buffer(b, n+2);
  if (log_data.tx_enabled) ucuart_tx_schedule(log_data.uart, NULL, 0);
}

size_t log_tx_avail(void) {
  return cb_write_avail(&tx_cb);
}

#define LOG_APP_HASH_SIZE 64
const uint8_t* log_app_hash(size_t* n) {
#ifdef CONFIG_UC_SIGNED_IMAGE
  const uint8_t *app_hash__ = sbl_app_hash();
#else
// Will be initialized by apphash.cmake
static const __attribute__((section(".apphash"))) uint8_t app_hash__[LOG_APP_HASH_SIZE];
#endif
  if (n != NULL) *n = LOG_APP_HASH_SIZE;
  return app_hash__;
}

#if CONFIG_UC_LOG_SAVE

static NOCLEAR uint8_t app_hash[LOG_APP_HASH_SIZE];

static uint8_t saved_app_hash[LOG_APP_HASH_SIZE];
static uint8_t saved_log[CONFIG_UC_LOG_BUF_SIZE];
static size_t saved_log_n;

const uint8_t* log_saved_log(size_t* n) {
  *n = saved_log_n;
  return saved_log;
}

const uint8_t* log_saved_app_hash(size_t* n) {
  *n = LOG_APP_HASH_SIZE;
  return saved_app_hash;
}

// app_hash__ and app_hash will only be different on a code change.
// We don't want previous log details for code changes.
static bool log_valid(void) {
  return (tx_cb.write < tx_cb.n)     && (tx_cb.read < tx_cb.n) &&
         (tx_cb.n == sizeof(tx_buf)) && (tx_cb.b == tx_buf) &&
         (memcmp(log_app_hash(NULL), app_hash, sizeof(app_hash)) == 0);
}

static void log_save(void) {
  saved_log_n = cb_read_avail(&tx_cb);

  // If it is empty then "force" dumping the entire contents
  if (saved_log_n == 0) {
    cb_skip(&tx_cb, 1);
    saved_log_n = cb_read_avail(&tx_cb);
  }
  if (saved_log_n > sizeof(saved_log)) saved_log_n = sizeof(saved_log);

  uint8_t* save = saved_log;
  size_t log_rem = saved_log_n;
  size_t n = cb_peek_avail(&tx_cb);
  if (n > log_rem) n = log_rem;
  memmove(save, cb_peek(&tx_cb), n);
  save += n;
  cb_skip(&tx_cb, n);
  log_rem -= n;
  n = cb_peek_avail(&tx_cb);
  if (n > log_rem) n = log_rem;
  memmove(save, cb_peek(&tx_cb), n);
  cb_skip(&tx_cb, n);

  // Save the app_hash associated with the log.
  memmove(saved_app_hash, app_hash, sizeof(app_hash));
}

static void log_save_init(void) {
  saved_log_n = 0;
  if (log_valid()) log_save();

  // Save current app hash so we can get access on next reset
  // app_hash__ might change between resets.
  memmove(app_hash, log_app_hash(NULL), sizeof(app_hash));
}

#else

static void log_save_init(void) {
}

#endif

void log_pre_init(void) {
  log_save_init();

  log_data.uart = NULL;
  memset(tx_buf, 0, sizeof(tx_buf));
  cb_init(&tx_cb, tx_buf, sizeof(tx_buf));
  log_tx_suspend();
  LOG_INFO("log-pre-init");
}

void log_init(uart_t* uart) {
  if (uart == NULL) return;

#ifdef CONFIG_APP_VERSION
  fill_device_info();
#endif

  log_data.uart = uart;
  ucuart_set_tx_cb(log_data.uart, &tx_cb);
#if !defined(CONFIG_UC_LOG_SERVER)
  // If there is no server then assume we can send at all times after init
  // completes.
  log_tx_resume();
#endif
}

#if defined(CONFIG_LOG_CUSTOM_HEADER)

#include <zephyr/sys/libc-hooks.h>
#include <zephyr/sys/printk-hooks.h>

// TODO Switch to using port 0 for stdin/stdout like stuff
// TODO hook stdin as well as stdout
// With semihosting the hooks are different as we take over low level write/read
// Perhaps some conflict with pico libc.

#if defined(CONFIG_STDOUT_CONSOLE) || defined(CONFIG_PRINTK)

static char line[100-3-1]; // To allow for log_logn_ overheads
static size_t  line_idx = 0;
static int console_out(int c) {
  if (line_idx < sizeof(line)-1) {
    if (c == '\n') {
      line[line_idx] = '\0';
      LOG_INFO("%s", line);
      line_idx = 0;
    }
    else {
      line[line_idx++] = c;
    }
  }
  else {
    line[line_idx] = '\0';
    LOG_INFO("%s", line);
    line_idx = 0;
  }
  return c;
}

#endif

int zephyr_log_pre_init(void) {
  log_pre_init();

#if defined(CONFIG_STDOUT_CONSOLE)
  __stdout_hook_install(console_out);
#endif
#if defined(CONFIG_PRINTK)
  __printk_hook_install(console_out);
#endif
  return 0;
}

SYS_INIT(zephyr_log_pre_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static const struct device* console = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_console));

int zephyr_log_init(void) {
  if (!device_is_ready(console)) return -ENOTSUP;
  log_init(console);
  return 0;
}

SYS_INIT(zephyr_log_init, APPLICATION, 10);
#endif

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

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "ucuart.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(CONFIG_UC_LOG_SERVER)
#define CONFIG_UC_LOG_SERVER (0)
#endif

#if !defined(CONFIG_UC_LOG_SAVE)
#define CONFIG_UC_LOG_SAVE (0)
#endif

#if !defined(LOG_MAX_PACKET_SIZE)
#define LOG_MAX_PACKET_SIZE (1500)
#endif

// ports for ucLog infrastructure
#define LOG_PORT_PLOTTING 60
#define LOG_PORT_SHELL 61
#define LOG_PORT_DEVICE_INFO 62
#define LOG_PORT_SYSCALLS 63
#define LOG_MAX_PORTS 64
#define NUM_INTERNAL_LOG_PORTS (LOG_MAX_PORTS - LOG_PORT_PLOTTING)

// Override/overload zephyr logging macros
#if defined(CONFIG_LOG_CUSTOM_HEADER)
#undef LOG_ERR
#undef LOG_WRN
#undef LOG_INF
#undef LOG_DBG
#undef LOG_RAW
#undef LOG_WRN_ONCE

#undef LOG_HEXDUMP_ERR
#undef LOG_HEXDUMP_WRN
#undef LOG_HEXDUMP_INF
#undef LOG_HEXDUMP_DBG

#define LOG_ERR(...)                                                           \
  do {                                                                         \
    if (Z_LOG_CONST_LEVEL_CHECK(LOG_LEVEL_ERR)) {                              \
      LOG_ERROR(__VA_ARGS__);                                                  \
    }                                                                          \
  } while (0)
#define LOG_WRN(...)                                                           \
  do {                                                                         \
    if (Z_LOG_CONST_LEVEL_CHECK(LOG_LEVEL_WRN)) {                              \
      LOG_WARN(__VA_ARGS__);                                                   \
    }                                                                          \
  } while (0)
#define LOG_INF(...)                                                           \
  do {                                                                         \
    if (Z_LOG_CONST_LEVEL_CHECK(LOG_LEVEL_INF)) {                              \
      LOG_INFO(__VA_ARGS__);                                                   \
    }                                                                          \
  } while (0)
#define LOG_DBG(...) do { \
  if (Z_LOG_CONST_LEVEL_CHECK(LOG_LEVEL_DBG)) { \
    LOG_DEBUG(__VA_ARGS__);  \
  } \
} while (0)

#define LOG_WRN_ONCE(...) do { \
  static uint8_t warned__ = 0; \
  if (warned__ == 0) { \
    LOG_WRN(__VA_ARGS__); \
    warned__ = 1; \
  } \
} while (0)

#define LOG_RAW(...) LOG_ERROR("LOG_RAW - not supported")

#define LOG_HEXDUMP_ERR(data_, length_, str_) do { \
  if (Z_LOG_CONST_LEVEL_CHECK(LOG_LEVEL_ERR)) { \
    LOG_MEM_ERROR(str_, data_, length_);  \
  } \
} while (0)
#define LOG_HEXDUMP_WRN(data_, length_, str_) do { \
  if (Z_LOG_CONST_LEVEL_CHECK(LOG_LEVEL_WRN)) { \
    LOG_MEM_WARN(str_, data_, length_);  \
  } \
} while (0)
#define LOG_HEXDUMP_INF(data_, length_, str_) do { \
  if (Z_LOG_CONST_LEVEL_CHECK(LOG_LEVEL_INF)) { \
    LOG_MEM_INFO(str_, data_, length_);  \
  } \
} while (0)
#define LOG_HEXDUMP_DBG(data_, length_, str_) do { \
  if (Z_LOG_CONST_LEVEL_CHECK(LOG_LEVEL_DBG)) { \
    LOG_MEM_DEBUG(str_, data_, length_);  \
  } \
} while (0)

#else

// Only need to define these if not zephyr
// On zephyr these are configured to run at startup
void log_pre_init(void);
void log_init(uart_t* uart);

#endif

// NOTE:
//
//   Logging code assumes that __FILE__ contains no colons.
//
// If __FILE__ contains any ':'s then the decoder will not be able to
// correctly parse the generated "database".

// Convert to using one printf like function by compile time
// computing a string of format characters that can be used with
// a simple switch/var_arg loop.  That way we have parsed the format string
// (as C compiler will check argument types), and we can use %s.
// For enums and other "special decode forms" use the following syntax:
//   {enum:<enumname> %u}  - uses value to look up corresponding enum string

#define VA_NARGS_IMPL(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, N, ...) N
#define VA_CNT(...) VA_NARGS_IMPL(__VA_ARGS__, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)
#define VA_SEL(...) VA_NARGS_IMPL(__VA_ARGS__, N, N, N, N, N, N, N, N, N, N, N, N, N, N, 1)

#define LOG_LVL_DEBUG 0
#define LOG_LVL_INFO  1
#define LOG_LVL_WARN  2
#define LOG_LVL_ERROR 3
#define LOG_LVL_FATAL 4
#define LOG_LVL_PANIC 5

#define TOSTR_(x_)  TOSTR1_(x_)
#define TOSTR1_(x_) #x_

#define LOG_STRING_(x_)                                                        \
  (__extension__({                                                             \
    static const                                                               \
        __attribute__((__aligned__(4),                                         \
                       __section__(".logstr." TOSTR_(__LINE__)))) char c__[] = \
            (x_);                                                              \
    (const char *)&c__;                                                        \
  }))

#define LOG_DEBUG(...) LOG_(LOG_LVL_DEBUG, VA_SEL(__VA_ARGS__), __VA_ARGS__)
#define LOG_INFO(...)  LOG_(LOG_LVL_INFO , VA_SEL(__VA_ARGS__), __VA_ARGS__)
#define LOG_WARN(...)  LOG_(LOG_LVL_WARN , VA_SEL(__VA_ARGS__), __VA_ARGS__)
#define LOG_ERROR(...) LOG_(LOG_LVL_ERROR, VA_SEL(__VA_ARGS__), __VA_ARGS__)
#define LOG_FATAL(...) do { \
  log_panic_(); \
  LOG_(LOG_LVL_FATAL, VA_SEL(__VA_ARGS__), __VA_ARGS__); \
  log_fatal_(); \
} while (false)

#define LOG_(c_,s_,...)      LOG_IMPL_(c_,s_,__VA_ARGS__)
#define LOG_IMPL_(c_,s_,...) LOG##s_##_(c_,__VA_ARGS__)

#define LOG1_(c_, fmt_)  \
  do { \
    log_fmt_chk_(fmt_); \
    log_log1_( \
      LOG_STRING_(#c_ ":" __FILE__ ":" TOSTR_(__LINE__) ":" fmt_)); \
  } while (false)

#define LOGN_(c_, fmt_, ...)  \
  do  { \
    const char mfmt_[] = { MAP(typechar, __VA_ARGS__) 0 }; \
    log_fmt_chk_(fmt_, __VA_ARGS__); \
    log_logn_(mfmt_, \
        LOG_STRING_(#c_ ":" __FILE__ ":" TOSTR_(__LINE__) ":" fmt_), \
        __VA_ARGS__); \
  } while (false)

#define LOG_MEM_DEBUG(_fmt, _buf, _n) LOG_MEM_(LOG_LVL_DEBUG, _fmt, _buf, _n)
#define LOG_MEM_INFO(_fmt, _buf, _n)  LOG_MEM_(LOG_LVL_INFO,  _fmt, _buf, _n)
#define LOG_MEM_WARN(_fmt, _buf, _n)  LOG_MEM_(LOG_LVL_WARN,  _fmt, _buf, _n)
#define LOG_MEM_ERROR(_fmt, _buf, _n) LOG_MEM_(LOG_LVL_ERROR, _fmt, _buf, _n)
#define LOG_MEM_FATAL(_fmt, _buf, _n) do { \
  log_panic_(); \
  LOG_MEM_(LOG_LVL_FATAL, _fmt, _buf, _n); \
  log_fatal_(); \
} while (false)
#define LOG_MEM_(_c, _fmt, _buf, _n) LOG_MEM_IMPL_(_c, _fmt, _buf, _n)

#define LOG_MEM_IMPL_(_c, _fmt, _buf, _n) \
  do { \
    log_mem_( \
      LOG_STRING_(#_c ":" __FILE__ ":" TOSTR_(__LINE__) ":" _fmt), _buf, _n); \
  } while (false)


static inline void __attribute__((always_inline, format(printf,1,2)))
  log_fmt_chk_(__attribute__((unused)) const char *fmt, ...) {}

typedef struct {
  uint8_t* rx;
  size_t   rx_n;
  uint8_t* tx;
  size_t   tx_n;
} log_msg_t;

void log_log1_(const char *prefix);
void log_logn_(const char* n, const char *prefix,  ...);
void log_mem_(const char *prefix,  const void* b, size_t n);

void log_panic_(void);
__attribute__((noreturn)) void log_fatal_(void);

void log_tx(uint8_t port, const uint8_t* data, size_t n);
size_t log_tx_avail(void);

void log_tx_suspend(void);
void log_tx_resume(void);

int log_is_host_ready(bool *host_ready);

#if CONFIG_UC_LOG_SERVER
size_t log_rx(uint8_t port, uint8_t* data, size_t n);
typedef void log_cb_t(const uint8_t* rx, size_t rx_n, void* ctx);
void log_notify(uint8_t port, log_cb_t* task, void* ctx);
#endif

#define LOG_APP_HASH_SIZE 64
const uint8_t* log_app_hash(size_t* n);

#if CONFIG_UC_LOG_SAVE
const uint8_t* log_saved_log(size_t* n);
const uint8_t* log_saved_app_hash(size_t* n);
void log_tx_saved_log(void);
#endif


// MAP that takes up to 21 arguments (can be adjusted upwards by updating
// ARG_CNT_ and HAS_COMMA)
//
// Uses a combination of the techniques in:
//  - https://gustedt.wordpress.com/2010/06/08/detect-empty-macro-arguments/
//  - http://jhnet.co.uk/articles/cpp_magic
//
// The approach in the second article for detecting the empty list doens't
// work on the following example
//    EVAL(MAP(GREET, Mum, (_Bool) Dad))
// The cast confuses the test for empty list.

#define ARG_CNT_(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, ...) _20
#define HAS_COMMA(...) ARG_CNT_(__VA_ARGS__, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0)
#define TRIGGER_PARENTHESIS_(...) ,

#define ISEMPTY(...)                                                    \
ISEMPTY_(                                                               \
          /* test if there is just one argument, eventually an empty    \
             one */                                                     \
          HAS_COMMA(__VA_ARGS__),                                       \
          /* test if TRIGGER_PARENTHESIS_ together with the argument   \
             adds a comma */                                            \
          HAS_COMMA(TRIGGER_PARENTHESIS_ __VA_ARGS__),                 \
          /* test if the argument together with a parenthesis           \
             adds a comma */                                            \
          HAS_COMMA(__VA_ARGS__ (/*empty*/)),                           \
          /* test if placing it between TRIGGER_PARENTHESIS_ and the   \
             parenthesis adds a comma */                                \
          HAS_COMMA(TRIGGER_PARENTHESIS_ __VA_ARGS__ (/*empty*/))      \
          )

#define PASTE5(_0, _1, _2, _3, _4) _0 ## _1 ## _2 ## _3 ## _4
#define ISEMPTY_(_0, _1, _2, _3) HAS_COMMA(PASTE5(IS_EMPTY_CASE_, _0, _1, _2, _3))
#define IS_EMPTY_CASE_0001 ,

#define UCEMPTY()

#define EVAL(...) EVAL32(__VA_ARGS__)
//#define EVAL1024(...) EVAL512(EVAL512(__VA_ARGS__))
//#define EVAL512(...) EVAL256(EVAL256(__VA_ARGS__))
//#define EVAL256(...) EVAL128(EVAL128(__VA_ARGS__))
//#define EVAL128(...) EVAL64(EVAL64(__VA_ARGS__))
#define EVAL64(...) EVAL32(EVAL32(__VA_ARGS__))
#define EVAL32(...) EVAL16(EVAL16(__VA_ARGS__))
#define EVAL16(...) EVAL8(EVAL8(__VA_ARGS__))
#define EVAL8(...) EVAL4(EVAL4(__VA_ARGS__))
#define EVAL4(...) EVAL2(EVAL2(__VA_ARGS__))
#define EVAL2(...) EVAL1(EVAL1(__VA_ARGS__))
#define EVAL1(...) __VA_ARGS__

#define DEFER1(m) m UCEMPTY()
#define DEFER2(m) m UCEMPTY UCEMPTY()()
#define DEFER3(m) m UCEMPTY UCEMPTY UCEMPTY()()()
#define DEFER4(m) m UCEMPTY UCEMPTY UCEMPTY UCEMPTY()()()()

#define CAT(a,b) a ## b

#define IF_NOT(condition) CAT(IF2_, condition)
#define IF2_1(...)
#define IF2_0(...) __VA_ARGS__

#define MAP_(m, first, ...)           \
  m(first)                           \
  IF_NOT(ISEMPTY(__VA_ARGS__))(    \
    DEFER2(MAP__)()(m, __VA_ARGS__)   \
  )
#define MAP__() MAP_

#define MAP(m, ...) EVAL(MAP_(m, __VA_ARGS__))

#ifdef __cplusplus
}
#endif

// The following depend on platform sizes
// '0' - 4 byte int - smaller things get promoted to this - sign doesn't matter
// '1' - 8 byte int
// '2' - double
// '3' - long double
// '4' - null terminated string
// '5' - pointer -  also the default if nothing else matches
// MacoS long int is 8
// ARM long int is 4

#ifdef __cplusplus

template <typename T> constexpr char cpp_typechar(T x) { return '5'; }
template <> constexpr char cpp_typechar<_Bool>(_Bool x) { return '0'; }
template <> constexpr char cpp_typechar<char>(char x) { return '0'; }
template <> constexpr char cpp_typechar<signed char>(signed char x) {
  return '0';
}
template <> constexpr char cpp_typechar<unsigned char>(unsigned char x) {
  return '0';
}
template <> constexpr char cpp_typechar<short int>(short int x) { return '0'; }
template <>
constexpr char cpp_typechar<unsigned short int>(unsigned short int x) {
  return '0';
}
template <> constexpr char cpp_typechar<int>(int x) { return '0'; }
template <> constexpr char cpp_typechar<unsigned int>(unsigned int x) {
  return '0';
}
template <> constexpr char cpp_typechar<long int>(long int x) { return '0'; }
template <>
constexpr char cpp_typechar<unsigned long int>(unsigned long int x) {
  return '0';
}
template <> constexpr char cpp_typechar<long long int>(long long int x) {
  return '1';
}
template <>
constexpr char cpp_typechar<unsigned long long int>(unsigned long long int x) {
  return '1';
}
template <> constexpr char cpp_typechar<float>(float x) { return '2'; }
template <> constexpr char cpp_typechar<double>(double x) { return '2'; }
template <> constexpr char cpp_typechar<long double>(long double x) {
  return '3';
}
template <> constexpr char cpp_typechar<char *>(char *x) { return '4'; }
template <> constexpr char cpp_typechar<const char *>(const char *x) {
  return '4';
}

#define typechar(x) cpp_typechar(x),

#else

#define typechar(x) _Generic((x), \
    _Bool:                  '0', \
    char:                   '0', \
    signed char:            '0', \
    unsigned char:          '0', \
    short int:              '0', \
    unsigned short int:     '0', \
    int:                    '0', \
    unsigned int:           '0', \
    long int:               '0', \
    unsigned long int:      '0', \
    long long int:          '1', \
    unsigned long long int: '1', \
    float:                  '2', \
    double:                 '2', \
    long double:            '3', \
    char *:                 '4', \
    const char *:           '4', \
    default:                '5'),

#endif

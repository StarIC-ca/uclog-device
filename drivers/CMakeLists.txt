# © 2025 Unit Circle Inc.
# SPDX-License-Identifier: Apache-2.0

#zephyr_syscall_header_ifdef(CONFIG_UCUART ${UCLOG_ROOT_DIR}/include/ucuart.h)
zephyr_syscall_header(${UCLOG_ROOT_DIR}/include/ucuart.h)

add_subdirectory_ifdef(CONFIG_UCUSB ucusb)
add_subdirectory_ifdef(CONFIG_UCUART ucuart)
add_subdirectory_ifdef(CONFIG_UCUART_Z ucuart_z)

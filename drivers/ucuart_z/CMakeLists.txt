# © 2026 StarIC Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_UCUART_Z ucuart_z.c)

#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(homeflow_embedded_final)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/bme.c
  src/temp.c
  src/r_enc.c
  src/imu.c
  src/rc_filter.c
  src/hr.c
  src/display.c
  src/ring_buf.c
  src/spi.c
  src/mutexes.c
)

# Include UART ASYNC API adapter
target_sources_ifdef(CONFIG_BT_NUS_UART_ASYNC_ADAPTER app PRIVATE
  src/uart_async_adapter.c
)

# NORDIC SDK APP END

zephyr_library_include_directories(.)

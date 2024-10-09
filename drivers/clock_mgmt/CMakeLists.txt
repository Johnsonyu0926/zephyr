# Copyright 2024 NXP
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_library()

zephyr_library_sources(clock_mgmt_common.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MGMT_FIXED_SOURCE fixed_clock_source.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_MGMT_SOURCE clock_source.c)
# Header for common clock drivers
add_clock_mgmt_header(clock_mgmt_drivers.h)

add_subdirectory(nxp_syscon)

# Include headers for all clock management drivers, registered with add_clock_mgmt_header
zephyr_library_compile_options("-include$<JOIN:$<TARGET_PROPERTY:clock_mgmt_header_target,COMPILE_DEFINITIONS>,;-include>")

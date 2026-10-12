# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

# LVGL is kept in this localized SDK so the application can build without
# fetching third-party sources at configure time.
file(GLOB_RECURSE LVGL_SRC CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
)

sdk_inc(${CMAKE_CURRENT_LIST_DIR})
sdk_src(${LVGL_SRC})

# The HPM display/tick port is intentionally not enabled here.  The motor
# board does not define a panel or touch controller yet; applications can add
# their own port by setting CONFIG_LVGL_CUSTOM_PORTABLE=1 (the default below).

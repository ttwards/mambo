# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

# FIXME: SHADOW_VARS: Remove this once we have enabled -Wshadow globally.
# add_compile_options($<TARGET_PROPERTY:compiler,warning_shadow_variables>)

# add_definitions(-D__ZEPHYR_SUPERVISOR__)

add_subdirectory_ifdef(CONFIG_MOTOR_COMMON motor)

add_subdirectory_ifdef(CONFIG_PID pid)

add_subdirectory_ifdef(CONFIG_ARES_SBUS transfer)

add_subdirectory_ifdef(CONFIG_CHASSIS chassis)

add_subdirectory_ifdef(CONFIG_WHEEL wheel)

add_subdirectory_ifdef(CONFIG_SENSOR sensor)

add_subdirectory_ifdef(CONFIG_VCAN vcan)

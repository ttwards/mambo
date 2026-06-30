# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MOTOR_COMMON common.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_COMMON motor_can_sched.c)

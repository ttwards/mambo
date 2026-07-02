# Yinshi LA Series Micro Servo Electric Cylinder driver
#
# Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_LA_YINSHI la_yinshi.c)

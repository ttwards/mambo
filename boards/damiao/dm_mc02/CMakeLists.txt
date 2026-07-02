# SPDX-License-Identifier: Apache-2.0

zephyr_library()
set(ARES_BOARD_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../lib/ares/board)
zephyr_library_sources(${ARES_BOARD_DIR}/init.c ${ARES_BOARD_DIR}/dm_mc02.c)

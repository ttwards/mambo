# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_CXX_STANDARD 17)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dm_can_sched_demo)

FILE(GLOB app_sources src/*.c)

include_directories(${ZEPHYR_BASE}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dts)

target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/motor/common/motor_can_sched.c)

target_include_directories(app PRIVATE ${ZEPHYR_BASE}/include)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dts)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/motor/common)

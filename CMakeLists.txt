# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(BOARD f303re)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)


file(GLOB C_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
                ${CMAKE_CURRENT_SOURCE_DIR}/interface/*.c
                ${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.c
                ${CMAKE_CURRENT_SOURCE_DIR}/components/at_client/*.c)

set(C_INC ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/interface
          ${CMAKE_CURRENT_SOURCE_DIR}/drivers
          ${CMAKE_CURRENT_SOURCE_DIR}/components/at_client)

target_sources(app PRIVATE ${C_SRC})
target_include_directories(app PRIVATE ${C_INC})
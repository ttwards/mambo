# WS2812B LED Strip Library
#
# 使用方式：
#   在你的 app CMakeLists.txt 中（find_package(Zephyr) 之后）加入：
#
#     target_include_directories(app PRIVATE
#         ${CMAKE_CURRENT_SOURCE_DIR}/../ws2812_strip/include)
#     target_sources(app PRIVATE
#         ${CMAKE_CURRENT_SOURCE_DIR}/../ws2812_strip/ws2812_strip.c)
#
#   或者在 app CMakeLists.txt 中 add_subdirectory 此包：
#
#     add_subdirectory(../ws2812_strip ws2812_strip_build)

if(CONFIG_WS2812_STRIP)
zephyr_library()
zephyr_library_sources(ws2812_strip.c)
zephyr_include_directories(include)
endif()

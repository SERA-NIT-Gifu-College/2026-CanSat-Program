cmake_minimum_required(VERSION 3.14)

set(PICO_BOARD pico2_w)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(cansat2026 LANGUAGES C CXX ASM)

pico_sdk_init()

set(SRCS
    src/imu.c
    src/light_sensor.c
    src/main.c
)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE ${SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_gpio
    hardware_dma
    hardware_adc
    hardware_irq
    hardware_i2c
    hardware_pwm
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    WIFI_SSID=\"$ENV{WIFI_SSID}\"
    WIFI_PASSWD=\"$ENV{WIFI_PASSWD}\"
)

pico_enable_stdio_usb(${PROJECT_NAME} ON)
pico_enable_stdio_uart(${PROJECT_NAME} OFF)

pico_add_extra_outputs(${PROJECT_NAME})

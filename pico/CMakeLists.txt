cmake_minimum_required(VERSION 3.14)

set(PICO_BOARD pico2_w)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(cansat2026 LANGUAGES C CXX ASM)

pico_sdk_init()

set(SRCS
    src/button.c
    src/chip_enable.c
    src/imu.c
    src/light_sensor.c
    src/main.c
    src/motor.c
    src/server.c
    src/solar.c
)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE ${SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    hardware_adc
    hardware_dma
    hardware_flash
    hardware_gpio
    hardware_i2c
    hardware_irq
    hardware_pwm
    hardware_sync
    hardware_timer
    hardware_uart
    hardware_watchdog
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
    pico_flash
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    WIFI_SSID=\"$ENV{WIFI_SSID}\"
    WIFI_PASSWD=\"$ENV{WIFI_PASSWD}\"
)

pico_enable_stdio_usb(${PROJECT_NAME} OFF)
pico_enable_stdio_uart(${PROJECT_NAME} ON)

pico_add_extra_outputs(${PROJECT_NAME})

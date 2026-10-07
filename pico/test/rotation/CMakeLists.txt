project(cansat-test_rotation LANGUAGES C CXX ASM)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE
    test/rotation/main.c
    src/chip_enable.c
    src/server.c
    src/motor.c
    src/imu.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ./include
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    hardware_gpio
    hardware_i2c
    hardware_pwm
    hardware_timer
    hardware_uart
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    WIFI_SSID=\"$ENV{WIFI_SSID}\"
    WIFI_PASSWD=\"$ENV{WIFI_PASSWD}\"
)

pico_enable_stdio_usb(${PROJECT_NAME} OFF)
pico_enable_stdio_uart(${PROJECT_NAME} ON)

pico_add_extra_outputs(${PROJECT_NAME})

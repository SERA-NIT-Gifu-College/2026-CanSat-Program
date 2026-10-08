project(cansat-test_axis-remap LANGUAGES C CXX ASM)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE
    test/axis-remap/main.c
    src/chip_enable.c
    src/imu.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ./include
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    hardware_gpio
    hardware_i2c
    hardware_uart
    hardware_flash
    pico_stdlib
)

pico_enable_stdio_usb(${PROJECT_NAME} ON)
pico_enable_stdio_uart(${PROJECT_NAME} OFF)

pico_add_extra_outputs(${PROJECT_NAME})

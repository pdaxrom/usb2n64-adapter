cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(usb2n64_adapter)

pico_sdk_init()

add_executable(usb2n64_adapter)

pico_generate_pio_header(usb2n64_adapter ${CMAKE_CURRENT_LIST_DIR}/n64send.pio)

target_sources(usb2n64_adapter PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        ${CMAKE_CURRENT_LIST_DIR}/hid_app.c
        ${CMAKE_CURRENT_LIST_DIR}/hid_parser.c
        )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(usb2n64_adapter PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(usb2n64_adapter PUBLIC pico_stdlib pico_multicore hardware_pio hardware_dma hardware_flash tinyusb_host tinyusb_board)

pico_enable_stdio_usb(usb2n64_adapter 0)
pico_enable_stdio_uart(usb2n64_adapter 1)

pico_add_extra_outputs(usb2n64_adapter)

cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(usb2n64_adapter)

pico_sdk_init()

add_executable(usb2n64_adapter)

# Example source
target_sources(usb2n64_adapter PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(usb2n64_adapter PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_host
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(usb2n64_adapter PUBLIC pico_stdlib pico_multicore tinyusb_host tinyusb_board)

# enable usb output, disable uart output
pico_enable_stdio_usb(usb2n64_adapter 0)
pico_enable_stdio_uart(usb2n64_adapter 1)

pico_add_extra_outputs(usb2n64_adapter)

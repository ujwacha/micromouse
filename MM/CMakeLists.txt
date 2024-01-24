cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_mouse C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(mouse
  src/main.c
  src/PID.c
  )

target_link_libraries(mouse
  pico_multicore
  pico_stdlib
  hardware_uart
  hardware_pwm
  )

pico_enable_stdio_usb(mouse 1)
pico_enable_stdio_uart(mouse 1)


pico_add_extra_outputs(mouse )

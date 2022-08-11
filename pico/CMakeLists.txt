cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico-robovisor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_subdirectory(pwm)
add_subdirectory(i2c)
add_subdirectory(encoder)
add_subdirectory(pio_rotary_encoder)

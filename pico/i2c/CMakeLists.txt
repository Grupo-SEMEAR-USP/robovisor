add_executable(i2c
i2c.cpp
)

pico_enable_stdio_usb(i2c 1)
pico_enable_stdio_uart(i2c 1)
pico_add_extra_outputs(i2c)

target_link_libraries(i2c pico_stdlib hardware_i2c pico_binary_info)
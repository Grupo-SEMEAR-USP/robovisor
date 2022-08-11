add_executable(encoder
encoder.c
)

pico_enable_stdio_usb(encoder 1)
pico_enable_stdio_uart(encoder 1)
pico_add_extra_outputs(encoder)

pico_add_extra_outputs(encoder)

target_link_libraries(encoder pico_stdlib hardware_pio hardware_gpio)
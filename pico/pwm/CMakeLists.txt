add_executable(pwm
  pwm.c
)

pico_enable_stdio_usb(pwm 1)
pico_enable_stdio_uart(pwm 1)
pico_add_extra_outputs(pwm)

target_link_libraries(pwm pico_stdlib hardware_gpio hardware_pwm)
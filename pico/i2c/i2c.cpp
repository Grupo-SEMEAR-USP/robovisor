#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
 
#define PICO_SDA_5V 6
#define PICO_SCL_5V 7
#define PICO_SDA_0 8
#define PICO_SCL_0 9

#define i2c_default i2c1
#define I2C_ADDR 0x3E
 
int main() {
    // Enable UART so we can print status output
    stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_SDA_5V) || !defined(PICO_SCL_5V)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    i2c_set_slave_mode(i2c_default, true, I2C_ADDR);
    gpio_set_function(PICO_SDA_5V, GPIO_FUNC_I2C);
    gpio_set_function(PICO_SCL_5V, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_SDA_5V);
    gpio_pull_up(PICO_SCL_5V);

    gpio_init (PICO_SDA_0);
    gpio_set_dir (PICO_SDA_0, GPIO_OUT);

    gpio_init (PICO_SCL_0);
    gpio_set_dir (PICO_SCL_0, GPIO_OUT);

    gpio_set_outover(PICO_SDA_0, GPIO_OVERRIDE_LOW);
    gpio_set_outover(PICO_SCL_0, GPIO_OVERRIDE_LOW);

    // Make the I2C pins available to picotool
    uint8_t rxdata[4];
    uint8_t txdata[2];
    char message[20];
    while (true) {
        // Receive data from controller
        // 3 bytes received - byte 0 is cmd (used as lower byte) byte 2 is higher - byte 3 is 0
        if (i2c_get_read_available(i2c_default) < 3) continue;
        i2c_read_raw_blocking (i2c_default, rxdata, 3);
        sprintf (message, "Rx: %d %d %d\r\n", rxdata[0], rxdata[1], rxdata[2]);

        sprintf (message, "Value %d\r\n", rxdata[0]+(rxdata[1]<<8));
        
        // Respond with ADC value (in milivolts)
        uint16_t adc_value= 10;
        // Note that this will drop fraction rather than round, but close enough
        int value = (int) adc_value
        txdata[0] = value & 0xFF;
        txdata[1] = value >> 8;
        sprintf (message, "Tx: %d %d - %d\r\n", txdata[0], txdata[1], value);

        // Sends data in mv (as int)
        i2c_write_raw_blocking(i2c_default, txdata, 2);
        
    }
#endif
}
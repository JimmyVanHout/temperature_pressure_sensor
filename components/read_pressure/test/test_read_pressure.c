#include "read_pressure.h"

bool test_init_i2c_bus_and_controller() {
    /* define I2C settings */
    I2CSettings i2c_settings = {
        .I2C_CLOCK_SOURCE = I2C_CLK_SRC_DEFAULT, /* clock source (default is defined by I2C_CLK_SRC_DEFAULT macro) */
        .I2C_PORT = 0, /* I2C port number (default is 0) */
        .I2C_SCL_GPIO = 12, /* GPIO for I2C SCL */
        .I2C_SDA_GPIO = 14, /* GPIO for I2C SDA */
        .GLITCH_IGNORE_COUNT = 7, /* glitch ignore count of master bus (default is 7) */
        .INTERNAL_PULLUP_RESISTOR_STATE = 0, /* disable internal pullup resistor */
        .I2C_PERIPHERAL_ADDRESS_BIT_LENGTH = I2C_ADDR_BIT_LEN_7, /* I2C address bit length (7) of peripheral, defined by I2C_ADDR_BIT_LEN_7 macro */
        .PERIPHERAL_ADDRESS = 0b1110110, /* I2C address of peripheral */
        .SCL_FREQUENCY = 100, /* I2C SCL line frequency (kHz)*/
        .I2C_CONTROLLER_TIMEOUT = 1000, /* I2C controller timeout period (ms) during which master will await a response from peripheral */
        .I2C_PERIPHERAL_RESET = 0x1e, /* I2C peripheral reset command */
        .I2C_PERIPHERAL_PROM_READ_PREFIX = 0b1010, /* I2C peripheral PROM read command prefix */
        .I2C_MAX_RETRIES = 2, /* max number of retries after receiving error from an I2C function before restarting */
        .I2C_RETRY_DELAY = 100 / portTICK_PERIOD_MS, /* delay (ms) before reattempting I2C function after receiving an error, if retries remain, where portTICK_PERIOD_MS is a constant included from a FreeRTOS header */
    };

    i2c_master_bus_handle_t *bus_handle;
    i2c_master_dev_handle_t *i2c_controller_handle;
    init_i2c_bus_and_controller(bus_handle, i2c_controller_handle);
    return true;
}

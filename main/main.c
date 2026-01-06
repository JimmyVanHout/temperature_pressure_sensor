#include <stdint.h>
#include <stdio.h>
#include "read_temperature_pressure.h"

void app_main(void) {
    I2CSettings i2c_settings; /* I2C settings object initialized with the I2C settings */
    const uint8_t I2C_PORT = 0; /* I2C port */
    const uint8_t SCL_GPIO = 19; /* SCL GPIO port */
    const uint8_t SDA_GPIO = 21; /* SDA GPIO port */
    i2c_master_bus_handle_t i2c_bus_handle; /* handle for I2C bus */
    i2c_master_dev_handle_t i2c_controller_handle; /* handle for I2C controller */
    uint16_t calibration_coefficients[7]; /* calibration coefficients, CRC value, and factory settings */
    uint32_t uncompensated_temperature_and_pressure[2]; /* array containing the uncompensated temperature and pressure values */
    double compensated_temperature_and_pressure[2]; /* array containing the compensated temperature and pressure values */

    init_i2c_settings(&i2c_settings, I2C_PORT, SCL_GPIO, SDA_GPIO); /* initialize I2C settings */
    init_i2c_bus_and_controller(&i2c_bus_handle, &i2c_controller_handle, &i2c_settings); /* initialize I2C bus and controller */
    init_peripheral(i2c_controller_handle, calibration_coefficients, &i2c_settings); /* initialize peripheral and get calibration coefficients, CRC value, and factory settings */
    while (true) {
        read_uncompensated_temperature_and_pressure(i2c_controller_handle, uncompensated_temperature_and_pressure, &i2c_settings); /* read uncompensated temperature and pressure */
        calc_compensated_temperature_and_pressure(calibration_coefficients, uncompensated_temperature_and_pressure[0], uncompensated_temperature_and_pressure[1], compensated_temperature_and_pressure, &i2c_settings); /* calculate the compensated temperature and pressure */
        printf("compensated temperature (degrees C), compensated pressure (mbar): %lf, %lf\n", compensated_temperature_and_pressure[0], compensated_temperature_and_pressure[1]);
    }
}

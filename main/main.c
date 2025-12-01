#include "read_pressure.h"

void app_main(void) {
    i2c_master_bus_handle_t i2c_bus_handle; /* handle for I2C bus */
    i2c_master_dev_handle_t i2c_controller_handle; /* handle for I2C controller */
    uint16_t calibration_coefficients[7];
    uint32_t uncompensated_temperature_and_pressure[2];
    double compensated_temperature_and_pressure[2];
    init_i2c_bus_and_controller(&i2c_bus_handle, &i2c_controller_handle); /* initialize I2C bus and controller */
    init_peripheral(i2c_controller_handle, calibration_coefficients); /* initialize peripheral and get calibration coefficients, CRC value, and factory settings */
    while (true) {
        read_uncompensated_temperature_and_pressure(i2c_controller_handle, uncompensated_temperature_and_pressure); /* read uncompensated temperature and pressure */
        calc_compensated_temperature_and_pressure(calibration_coefficients, uncompensated_temperature_and_pressure[0], uncompensated_temperature_and_pressure[1], compensated_temperature_and_pressure); /* calculate the compensated temperature and pressure */
        printf("Compensated temperature (degrees C): %lf\n", compensated_temperature_and_pressure[0]);
        printf("Compensated pressure (mbar): %lf\n", compensated_temperature_and_pressure[1]);
    }
}

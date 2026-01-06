#ifndef READ_PRESSURE_H
#define READ_PRESSURE_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* I2C settings */
typedef struct i2c_settings {
    uint8_t i2c_clock_source; /* default clock source*/
    uint8_t i2c_port; /* I2C port number */
    uint8_t i2c_scl_gpio; /* GPIO for I2C SCL */
    uint8_t i2c_sda_gpio; /* GPIO for I2C SDA */
    uint8_t glitch_ignore_count; /* glitch ignore count of master bus */
    uint8_t internal_pullup_resistor_state; /* disable internal pullup resistor */
    uint8_t i2c_peripheral_address_bit_length; /* I2C address bit length of peripheral */
    uint8_t peripheral_address; /* I2C address of peripheral */
    uint8_t scl_frequency; /* I2C SCL line frequency (kHz)*/
    uint16_t i2c_controller_timeout; /* I2C controller timeout period (ms) during which master will await a response from peripheral */
    uint32_t i2c_peripheral_reset; /* I2C peripheral reset command */
    uint8_t i2c_peripheral_prom_read_prefix; /* I2C peripheral PROM read command prefix */
    uint8_t i2c_max_retries; /* max number of retries after receiving error from an I2C function before restarting */
    uint8_t i2c_retry_delay; /* delay (ms) before reattempting I2C function after receiving an error, if retries remain */
    uint8_t i2c_max_error_message_length; /* max length (char) of an I2C error message */
    uint8_t i2c_convert_pressure; /* I2C command to convert pressure from analog to digital via ADC using OSR = 4096 */
    uint8_t i2c_convert_temperature; /* I2C command to convert temperature from analog to digital via ADC using OSR = 4096 */
    float min_adc_wait_time; /* minimum wait time for ADC conversion of pressure and temperature using OSR = 4096 */
    uint8_t i2c_read_adc; /* I2C command to read uncompensated pressure or temperature data from ADC */
    bool i2c_status_messages; /* determines whether status messages should be printed to standard output (error messages will always be printed to standard error) */
} I2CSettings;

/*
Initialize I2C settings.

Arguments:
    I2CSettings *i2c_settings: pointer to an uninitialized I2CSettings object, which will be initialized with the default I2C settings
    uint8_t i2c_port: the I2C port number
    uint8_t i2c_scl_gpio: the I2C SCL GPIO port
    uint8_t i2c_sda_gpio: the I2C SDA GPIO port
Returns:
    (void) nothing
*/
void init_i2c_settings(I2CSettings *i2c_settings, uint8_t i2c_port, uint8_t i2c_scl_gpio, uint8_t i2c_sda_gpio);

/*
Initialize I2C bus and controller.

Arguments:
    i2c_master_bus_handle_t *i2c_bus_handle: pointer to the I2C bus handle
    i2c_master_dev_handle_t *i2c_controller_handle: pointer to the I2C controller handle
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void init_i2c_bus_and_controller(i2c_master_bus_handle_t *i2c_bus_handle, i2c_master_dev_handle_t *i2c_controller_handle, I2CSettings *i2c_settings);

/*
Write command to peripheral.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint8_t command: command to send to peripheral
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void write_to_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint8_t command, I2CSettings *i2c_settings);

/*
Read data from peripheral.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint8_t *read_buffer: pointer to array in which to store received data
    uint8_t read_buffer_length: length of read buffer
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void read_from_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint8_t *read_buffer, uint8_t read_buffer_length, I2CSettings *i2c_settings);

/*
Probe I2C peripheral.

Arguments:
    i2c_master_bus_handle_t i2c_bus_handle: I2C bus handle
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (bool) true if an error was received when probing the I2C peripheral, false otherwise
*/
bool probe_peripheral(i2c_master_bus_handle_t i2c_bus_handle, I2CSettings *i2c_settings);

/*
Initialize peripheral. Index 0 of the returned array consists of, from MSB to LSB: a 4 bit cyclic redundancy check value, the 7 bits 0b0100100, and 5 bit factory settings. Indexes 1 to 6 of the returned array contain the peripheral's calibration coefficients.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint16_t *calibration_coefficients: array of calibration coefficients, CRC value, and factory settings
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void init_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint16_t *calibration_coefficients, I2CSettings *i2c_settings);

/*
Read uncompensated temperature and pressure from peripheral. Uncompensated temperature and pressure are first converted from analog to digital by ADC before being read.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint32_t *uncompensated_temperature_and_pressure: array of size uint32_t and length 2 in which the uncompensated temperature and pressure values will be stored, with temperature at index 0 and pressure at index 1
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void read_uncompensated_temperature_and_pressure(i2c_master_dev_handle_t i2c_controller_handle, uint32_t *uncompensated_temperature_and_pressure, I2CSettings *i2c_settings);

/*
Calculate cyclic redundancy check (CRC) value according to algorithm provided in datasheet.

Arguments:
    uint16_t *prom_data: data from peripheral PROM, including CRC value, factory settings, and calibration coefficients
Returns:
    (uint8_t) CRC value
*/
uint8_t calc_crc(uint16_t *prom_data);

/*
Calculate compensated temperature and pressure based on algorithm in datasheet.

Arguments:
    uint16_t *calibration_coefficients: array of length 7 of data from PROM, including CRC value, factory settings, and calibration coefficients
    uint32_t uncompensated_temperature: the uncompensated temperature
    uint32_t uncompensated_pressure: the uncompensated pressure
    double *compensated_temperature_and_pressure: an array of length 2 in which to store the compensated temperature (at index 0) and compensated pressure (at index 1)
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void calc_compensated_temperature_and_pressure(uint16_t *calibration_coefficients, uint32_t uncompensated_temperature, uint32_t uncompensated_pressure, double *compensated_temperature_and_pressure, I2CSettings *i2c_settings);

#endif

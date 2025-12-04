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
    const uint8_t I2C_CLOCK_SOURCE; /* default clock source*/
    const uint8_t I2C_PORT; /* I2C port number */
    const uint8_t I2C_SCL_GPIO; /* GPIO for I2C SCL */
    const uint8_t I2C_SDA_GPIO; /* GPIO for I2C SDA */
    const uint8_t GLITCH_IGNORE_COUNT; /* glitch ignore count of master bus */
    const uint8_t INTERNAL_PULLUP_RESISTOR_STATE; /* disable internal pullup resistor */
    const uint8_t I2C_PERIPHERAL_ADDRESS_BIT_LENGTH; /* I2C address bit length of peripheral */
    const uint8_t PERIPHERAL_ADDRESS; /* I2C address of peripheral */
    const uint8_t SCL_FREQUENCY; /* I2C SCL line frequency (kHz)*/
    const uint16_t I2C_CONTROLLER_TIMEOUT; /* I2C controller timeout period (ms) during which master will await a response from peripheral */
    const uint32_t I2C_PERIPHERAL_RESET; /* I2C peripheral reset command */
    const uint8_t I2C_PERIPHERAL_PROM_READ_PREFIX; /* I2C peripheral PROM read command prefix */
    const uint8_t I2C_MAX_RETRIES; /* max number of retries after receiving error from an I2C function before restarting */
    const uint8_t I2C_RETRY_DELAY; /* delay (ms) before reattempting I2C function after receiving an error, if retries remain */
    const uint8_t I2C_MAX_ERROR_MESSAGE_LENGTH; /* max length (char) of an I2C error message */
    const uint8_t I2C_CONVERT_PRESSURE; /* I2C command to convert pressure from analog to digital via ADC using OSR = 4096 */
    const uint8_t I2C_CONVERT_TEMPERATURE; /* I2C command to convert temperature from analog to digital via ADC using OSR = 4096 */
    const float MIN_ADC_WAIT_TIME; /* minimum wait time for ADC conversion of pressure and temperature using OSR = 4096 */
    const uint8_t I2C_READ_ADC; /* I2C command to read uncompensated pressure or temperature data from ADC */
    const bool I2C_STATUS_MESSAGES; /* determines whether status messages should be printed to standard output (error messages will always be printed to standard error) */
} I2CSettings;

/*
Initialize I2C bus and controller.
Arguments:
    i2c_master_bus_handle_t *i2c_bus_handle: pointer to the I2C bus handle
    i2c_master_dev_handle_t *i2c_controller_handle: point to the I2C controller handle
Returns:
    (void) nothing
*/
void init_i2c_bus_and_controller(i2c_master_bus_handle_t *i2c_bus_handle, i2c_master_dev_handle_t *i2c_controller_handle);

/*
Write command to peripheral.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint8_t command: command to send to peripheral
Returns:
    (void) nothing
*/
void write_to_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint8_t command);

/*
Read data from peripheral.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint8_t *read_buffer: pointer to array in which to store received data
    uint8_t read_buffer_length: length of read buffer
Returns:
    (void) nothing
*/
void read_from_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint8_t *read_buffer, uint8_t read_buffer_length);

/*
Probe I2C peripheral.

Arguments:
    i2c_master_bus_handle_t i2c_bus_handle: I2C bus handle
Returns:
    (bool) true if an error was received when probing the I2C peripheral, false otherwise
*/
bool probe_peripheral(i2c_master_bus_handle_t i2c_bus_handle);

/*
Initialize peripheral. Index 0 of the returned array consists of, from MSB to LSB: a 4 bit cyclic redundancy check value, the 7 bits 0b0100100, and 5 bit factory settings. Indexes 1 to 6 of the returned array contain the peripheral's calibration coefficients.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint16_t *calibration_coefficients: array of calibration coefficients, CRC value, and factory settings
Returns:
    (void) nothing
*/
void init_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint16_t *calibration_coefficients);

/*
Read uncompensated temperature and pressure from peripheral. Uncompensated temperature and pressure are first converted from analog to digital by ADC before being read.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint32_t *uncompensated_temperature_and_pressure: array of size uint32_t and length 2 in which the uncompensated temperature and pressure values will be stored, with temperature at index 0 and pressure at index 1
Returns:
    (void) nothing
*/
void read_uncompensated_temperature_and_pressure(i2c_master_dev_handle_t i2c_controller_handle, uint32_t *uncompensated_temperature_and_pressure);

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
Returns:
    (void) nothing
*/
void calc_compensated_temperature_and_pressure(uint16_t *calibration_coefficients, uint32_t uncompensated_temperature, uint32_t uncompensated_pressure, double *compensated_temperature_and_pressure);

#endif

#include "read_pressure.h"

/* define I2C settings */
I2CSettings i2c_settings = {
    .I2C_CLOCK_SOURCE = I2C_CLK_SRC_DEFAULT, /* clock source (default is defined by I2C_CLK_SRC_DEFAULT macro) */
    .I2C_PORT = 0, /* I2C port number (default is 0) */
    .I2C_SCL_GPIO = 32, /* GPIO for I2C SCL */
    .I2C_SDA_GPIO = 33, /* GPIO for I2C SDA */
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
    .I2C_MAX_ERROR_MESSAGE_LENGTH = 100, /* max length (char) of an I2C error message */
    .I2C_CONVERT_PRESSURE = 0x48, /* I2C command to convert pressure from analog to digital via ADC using OSR = 4096 */
    .I2C_CONVERT_TEMPERATURE = 0x58, /* I2C command to convert temperature from analog to digital via ADC using OSR = 4096 */
    .MIN_ADC_WAIT_TIME = 8.61, /* minimum wait time (ms) for ADC conversion of pressure and temperature using OSR = 4096 */
    .I2C_READ_ADC = 0, /* I2C command to read uncompensated pressure or temperature data from ADC */
    .I2C_STATUS_MESSAGES = false, /* determines whether status messages should be printed to standard output (error messages will always be printed to standard error) */
};

/*
Initialize I2C bus and controller.
Arguments:
    i2c_master_bus_handle_t *i2c_bus_handle: pointer to the I2C bus handle
    i2c_master_dev_handle_t *i2c_controller_handle: point to the I2C controller handle
Returns:
    (void) nothing
*/
void init_i2c_bus_and_controller(i2c_master_bus_handle_t *i2c_bus_handle, i2c_master_dev_handle_t *i2c_controller_handle) {
    /* setup I2C bus */
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = i2c_settings.I2C_CLOCK_SOURCE, /* default clock source*/
        .i2c_port = i2c_settings.I2C_PORT, /* I2C port number */
        .scl_io_num = i2c_settings.I2C_SCL_GPIO, /* GPIO for I2C SCL */
        .sda_io_num = i2c_settings.I2C_SDA_GPIO, /* GPIO for I2C SDA */
        .glitch_ignore_cnt = i2c_settings. GLITCH_IGNORE_COUNT, /* glitch ignore period of master bus */
        .flags.enable_internal_pullup = i2c_settings.INTERNAL_PULLUP_RESISTOR_STATE, /* state (enabled or disabled) of internal pullup resistor */
    };
    esp_err_t error_code = i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle);
    const char *error = esp_err_to_name(error_code);
    uint8_t retries = i2c_settings.I2C_MAX_RETRIES;
    while (strcmp(error, "ESP_OK") != 0) {
        fprintf(stderr, "Error %s received while creating new I2C bus.\n", error);
        if (retries == 0) {
            fprintf(stderr, "Received errors after attempting to create new I2C bus %hhu times. Restarting...\n", i2c_settings.I2C_MAX_RETRIES);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings.I2C_RETRY_DELAY);
        error_code = i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle);
        error = esp_err_to_name(error_code);
    }
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Created new I2C bus.\n");
    }

    /* setup I2C controller */
    i2c_device_config_t i2c_controller_config = {
        .dev_addr_length = i2c_settings.I2C_PERIPHERAL_ADDRESS_BIT_LENGTH, /* I2C address bit length (7) of peripheral, defined by I2C_ADDR_BIT_LEN_7 macro */
        .device_address = i2c_settings.PERIPHERAL_ADDRESS, /* I2C address of peripheral */
        .scl_speed_hz = i2c_settings.SCL_FREQUENCY, /* I2C SCL line frequency (kHz)*/
    };
    error_code = i2c_master_bus_add_device(*i2c_bus_handle, &i2c_controller_config, i2c_controller_handle);
    error = esp_err_to_name(error_code);
    retries = i2c_settings.I2C_MAX_RETRIES;
    while (strcmp(error, "ESP_OK") != 0) {
        fprintf(stderr, "Error %s received while creating new I2C controller.\n", error);
        if (retries == 0) {
            fprintf(stderr, "Received errors after attempting to create new I2C controller %hhu times. Restarting...\n", i2c_settings.I2C_MAX_RETRIES);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings.I2C_RETRY_DELAY);
        error_code = i2c_master_bus_add_device(*i2c_bus_handle, &i2c_controller_config, i2c_controller_handle);
        error = esp_err_to_name(error_code);
    }
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Created new I2C controller.\n");
    }
}

/*
Write command to peripheral.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint8_t command: command to send to peripheral
Returns:
    (void) nothing
*/
void write_to_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint8_t command) {
    uint8_t write_buffer[1] = {command};
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Writing command %hhu to peripheral at address %hhu...\n", command, i2c_settings.PERIPHERAL_ADDRESS);
    }
    esp_err_t error_code = i2c_master_transmit(i2c_controller_handle, write_buffer, sizeof(uint8_t), i2c_settings.I2C_CONTROLLER_TIMEOUT);
    const char *error = esp_err_to_name(error_code);
    uint8_t retries = i2c_settings.I2C_MAX_RETRIES;
    while (strcmp(error, "ESP_OK") != 0) {
        fprintf(stderr, "Error %s received when writing command %hhu to peripheral.\n", error, command);
        if (retries == 0) {
            fprintf(stderr, "Received errors after attempting to write to peripheral %hhu times. Restarting...\n", i2c_settings.I2C_MAX_RETRIES);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings.I2C_RETRY_DELAY);
        error_code = i2c_master_transmit(i2c_controller_handle, write_buffer, sizeof(uint8_t), i2c_settings.I2C_CONTROLLER_TIMEOUT);
        error = esp_err_to_name(error_code);
    }
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Wrote command %hhu to peripheral.\n", command);
    }
}

/*
Read data from peripheral.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint8_t *read_buffer: pointer to array in which to store received data
    uint8_t read_buffer_length: length of read buffer
Returns:
    (void) nothing
*/
void read_from_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint8_t *read_buffer, uint8_t read_buffer_length) {
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Reading from peripheral...\n");
    }
    esp_err_t error_code = i2c_master_receive(i2c_controller_handle, read_buffer, sizeof(uint8_t) * read_buffer_length, i2c_settings.I2C_CONTROLLER_TIMEOUT);
    const char *error = esp_err_to_name(error_code);
    uint8_t retries = i2c_settings.I2C_MAX_RETRIES;
    while (strcmp(error, "ESP_OK") != 0) {
        fprintf(stderr, "Error %s received while reading from peripheral.\n", error);
        if (retries == 0) {
            fprintf(stderr, "Received errors after attempting to read from peripheral %hhu times. Restarting...\n", i2c_settings.I2C_MAX_RETRIES);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings.I2C_RETRY_DELAY);
        error_code = i2c_master_receive(i2c_controller_handle, read_buffer, sizeof(uint8_t) * read_buffer_length, i2c_settings.I2C_CONTROLLER_TIMEOUT);
        error = esp_err_to_name(error_code);
    }
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Read:\n");
        for (uint8_t i = 0; i < read_buffer_length; i++) {
            printf("\t%hhu\n", read_buffer[i]);
        }
        printf("from peripheral.\n");
    }
}

/*
Probe I2C peripheral.

Arguments:
    i2c_master_bus_handle_t i2c_bus_handle: I2C bus handle
Returns:
    (bool) true if an error was received when probing the I2C peripheral, false otherwise
*/
bool probe_peripheral(i2c_master_bus_handle_t i2c_bus_handle) {
    char *error = malloc(sizeof(char) * (i2c_settings.I2C_MAX_ERROR_MESSAGE_LENGTH + 1));
    bool error_received = false;
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Probing peripheral at address %hhu...\n", i2c_settings.PERIPHERAL_ADDRESS);
    }
    esp_err_t error_code = i2c_master_probe(i2c_bus_handle, (uint16_t) i2c_settings.PERIPHERAL_ADDRESS, i2c_settings.I2C_CONTROLLER_TIMEOUT);
    strcpy(error, esp_err_to_name(error_code));
    if (strcmp(error, "ESP_OK") == 0) {
        if (i2c_settings.I2C_STATUS_MESSAGES) {
            printf("Peripheral at address %hhu successfully probed with error status %s.\n", i2c_settings.PERIPHERAL_ADDRESS, error);
        }
    } else {
        error_received = true;
        fprintf(stderr, "Unsuccessful probe of peripheral at address %hhu with error status %s.\n", i2c_settings.PERIPHERAL_ADDRESS, error);
    }
    free(error);
    return error_received;
}

/*
Initialize peripheral. Index 0 of the returned array consists of, from MSB to LSB: a 4 bit cyclic redundancy check value, the 7 bits 0b0100100, and 5 bit factory settings. Indexes 1 to 6 of the returned array contain the peripheral's calibration coefficients.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint16_t *calibration_coefficients: array of calibration coefficients, CRC value, and factory settings
Returns:
    (void) nothing
*/
void init_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint16_t *calibration_coefficients) {
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Initializing peripheral...\n");
    }
    write_to_peripheral(i2c_controller_handle, i2c_settings.I2C_PERIPHERAL_RESET); /* reset peripheral */
    for (uint8_t i = 0; i < 7; i++) { /* read calibration coefficients from PROM at PROM addresses 0b001 to 0b110 */
        uint8_t read_buffer[2];
        write_to_peripheral(i2c_controller_handle, (i2c_settings.I2C_PERIPHERAL_PROM_READ_PREFIX << 4) | (i << 1)); /* read PROM from peripheral at PROM address */
        read_from_peripheral(i2c_controller_handle, read_buffer, 2);
        calibration_coefficients[i] = (read_buffer[0] << 8) | read_buffer[1];
    }
    uint8_t retries = i2c_settings.I2C_MAX_RETRIES;
    uint8_t crc_expected = calibration_coefficients[0] >> 12;
    uint8_t crc_calculated = calc_crc(calibration_coefficients);
    while (crc_expected ^ crc_calculated) { /* compare received CRC value from PROM with calculated CRC value */
        fprintf(stderr, "Calculated CRC %hhu did not match expected CRC %hhu.\n", crc_calculated, crc_expected);
        if (retries == 0) {
            fprintf(stderr, "Calculated CRC did not match expected CRC %hhu times. Restarting...\n", i2c_settings.I2C_MAX_RETRIES);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings.I2C_RETRY_DELAY);
        write_to_peripheral(i2c_controller_handle, i2c_settings.I2C_PERIPHERAL_RESET);
        for (uint8_t i = 0; i < 7; i++) { /* read calibration coefficients from PROM at PROM addresses 0b001 to 0b110 */
            uint8_t read_buffer[2];
            write_to_peripheral(i2c_controller_handle, (i2c_settings.I2C_PERIPHERAL_PROM_READ_PREFIX << 4) | (i << 1)); /* read PROM from peripheral at PROM address */
            read_from_peripheral(i2c_controller_handle, read_buffer, 2);
            calibration_coefficients[i] = (read_buffer[0] << 8) | read_buffer[1];
        }
        crc_expected = calibration_coefficients[0] >> 12;
        crc_calculated = calc_crc(calibration_coefficients);
    }
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("CRC passed.\n");
        printf("Successfully initialized peripheral.\n");
    }
}

/*
Read uncompensated temperature and pressure from peripheral. Uncompensated temperature and pressure are first converted from analog to digital by ADC before being read.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint32_t *uncompensated_temperature_and_pressure: array of size uint32_t and length 2 in which the uncompensated temperature and pressure values will be stored, with temperature at index 0 and pressure at index 1
Returns:
    (void) nothing
*/
void read_uncompensated_temperature_and_pressure(i2c_master_dev_handle_t i2c_controller_handle, uint32_t *uncompensated_temperature_and_pressure) {
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Reading uncompensated temperature...\n");
    }
    write_to_peripheral(i2c_controller_handle, i2c_settings.I2C_CONVERT_TEMPERATURE);
    vTaskDelay(i2c_settings.MIN_ADC_WAIT_TIME);
    write_to_peripheral(i2c_controller_handle, i2c_settings.I2C_READ_ADC);
    uint8_t read_buffer[3];
    read_from_peripheral(i2c_controller_handle, read_buffer, 3);
    uint32_t temperature = 0;
    temperature = temperature | read_buffer[0];
    for (uint8_t i = 1; i < 3; i++) {
        temperature = temperature << 8;
        temperature = temperature | read_buffer[i];
    }
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Successfully read uncompensated temperature: %lu\n", temperature);
    }
    uncompensated_temperature_and_pressure[0] = temperature;
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Reading uncompensated pressure...\n");
    }
    write_to_peripheral(i2c_controller_handle, i2c_settings.I2C_CONVERT_PRESSURE);
    vTaskDelay(i2c_settings.MIN_ADC_WAIT_TIME);
    write_to_peripheral(i2c_controller_handle, i2c_settings.I2C_READ_ADC);
    read_from_peripheral(i2c_controller_handle, read_buffer, 3);
    uint32_t pressure = 0;
    pressure = pressure | read_buffer[0];
    for (uint8_t i = 1; i < 3; i++) {
        pressure = pressure << 8;
        pressure = pressure | read_buffer[i];
    }
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Successfully read uncompensated pressure: %lu\n", pressure);
    }
    uncompensated_temperature_and_pressure[1] = pressure;
}

/*
Calculate cyclic redundancy check (CRC) value according to algorithm provided in datasheet.

Arguments:
    uint16_t *prom_data: data from peripheral PROM, including CRC value, factory settings, and calibration coefficients
Returns:
    (uint8_t) CRC value
*/
uint8_t calc_crc(uint16_t *prom_data) {
    uint16_t pd[8];
    for (uint8_t i = 0; i < 7; i++) {
        pd[i] = prom_data[i];
    }
    pd[0] = pd[0] & 0x0fff;
    pd[7] = 0;
    uint16_t r = 0;
    for (uint8_t c = 0; c < 16; c++) {
        if (c % 2) {
            r = r ^ (uint8_t) (pd[c >> 1] & 0x00ff);
        } else {
            r = r ^ (uint8_t) (pd[c >> 1] >> 8);
        }
        for (uint8_t b = 8; b > 0; b--) {
            if (r & 0x8000) {
                r = (r << 1) ^ 0x3000;
            } else {
                r = r << 1;
            }
        }
    }
    r = (r >> 12) & 0x000f;
    r = r ^ 0;
    return r;
}

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
void calc_compensated_temperature_and_pressure(uint16_t *calibration_coefficients, uint32_t uncompensated_temperature, uint32_t uncompensated_pressure, double *compensated_temperature_and_pressure) {
    /* first order temperature and pressure calculations */

    /* calculate first order temperature */
    double reference_temperature = calibration_coefficients[5];
    double temperature_coefficient_of_temperature = calibration_coefficients[6];
    double temp_diff = uncompensated_temperature - reference_temperature * 0x100;
    double compensated_temperature = 2000 + (temp_diff * temperature_coefficient_of_temperature) / (double) 0x800000;
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Compensated temperature first order: %lf\n", compensated_temperature);
    }

    /* calculate first order pressure */
    double pressure_offset = calibration_coefficients[2];
    double temperature_coefficient_of_pressure_offset = calibration_coefficients[4];
    double pressure_sensitivity = calibration_coefficients[1];
    double temperature_coefficient_of_pressure_sensitivity = calibration_coefficients[3];
    double offset_actual_temperature = pressure_offset * 0x20000 + (temp_diff * temperature_coefficient_of_pressure_offset) / (double) 0x40;
    double sensitivity_actual_temperature = pressure_sensitivity * 0x10000 + (temp_diff * temperature_coefficient_of_pressure_sensitivity) / (double) 0x80;
    double compensated_pressure = ((uncompensated_pressure * sensitivity_actual_temperature) / (double) 0x200000 - offset_actual_temperature) / (double) 0x8000;
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Compensated pressure first order: %lf\n", compensated_pressure);
    }

    /* second order temperature and pressure calculations */
    double ti, offi, sensi;
    double compensated_temperature_c = compensated_temperature / 100.0;
    double compensated_pressure_mbar = compensated_pressure / 100.0;
    if (compensated_temperature_c > 20) {
        ti = offi = sensi = 0;
    } else {
        if (compensated_temperature_c > 10) {
            ti = 12 * pow(temp_diff, 2) / (double) 0x800000000;
            offi = 30 * pow(compensated_temperature - 2000, 2) / (double) 0x100;
            sensi = 0;
        } else {
            ti = 14 * pow(temp_diff, 2) / (double) 0x800000000;
            offi = 35 * pow(compensated_temperature - 2000, 2) / (double) 0x8;
            sensi = 63 * pow(compensated_temperature - 2000, 2) / (double) 0x20;
        }
    }
    compensated_temperature_c = (compensated_temperature - ti) / 100.0;
    offset_actual_temperature -= offi;
    sensitivity_actual_temperature -= sensi;
    compensated_pressure_mbar = (((uncompensated_pressure * sensitivity_actual_temperature) / (double) 0x200000 - offset_actual_temperature) / (double) 0x8000) / 100.0;
    if (i2c_settings.I2C_STATUS_MESSAGES) {
        printf("Compensated temperature second order: %lf\n", compensated_temperature_c);
        printf("Compensated pressure second order: %lf\n", compensated_pressure_mbar);
    }
    compensated_temperature_and_pressure[0] = compensated_temperature_c;
    compensated_temperature_and_pressure[1] = compensated_pressure_mbar;
}

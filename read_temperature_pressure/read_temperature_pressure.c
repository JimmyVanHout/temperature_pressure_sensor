#include "read_temperature_pressure.h"

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
void init_i2c_settings(I2CSettings *i2c_settings, uint8_t i2c_port, uint8_t i2c_scl_gpio, uint8_t i2c_sda_gpio) {
    i2c_settings->i2c_clock_source = I2C_CLK_SRC_DEFAULT; /* clock source (default is defined by I2C_CLK_SRC_DEFAULT macro) */
    i2c_settings->i2c_port = i2c_port; /* I2C port number */
    i2c_settings->i2c_scl_gpio = i2c_scl_gpio; /* GPIO for I2C SCL */
    i2c_settings->i2c_sda_gpio = i2c_sda_gpio; /* GPIO for I2C SDA */
    i2c_settings->glitch_ignore_count = 7; /* glitch ignore count of master bus (default is 7) */
    i2c_settings->internal_pullup_resistor_state = 0; /* disable internal pullup resistor */
    i2c_settings->i2c_peripheral_address_bit_length = I2C_ADDR_BIT_LEN_7; /* I2C address bit length (7) of peripheral, defined by I2C_ADDR_BIT_LEN_7 macro */
    i2c_settings->peripheral_address = 0b1110110; /* I2C address of peripheral */
    i2c_settings->scl_frequency = 100; /* I2C SCL line frequency (kHz)*/
    i2c_settings->i2c_controller_timeout = 1000; /* I2C controller timeout period (ms) during which master will await a response from peripheral */
    i2c_settings->i2c_peripheral_reset = 0x1e; /* I2C peripheral reset command */
    i2c_settings->i2c_peripheral_prom_read_prefix = 0b1010; /* I2C peripheral PROM read command prefix */
    i2c_settings->i2c_max_retries = 2; /* max number of retries after receiving error from an I2C function before restarting */
    i2c_settings->i2c_retry_delay = 100 / portTICK_PERIOD_MS; /* delay (ms) before reattempting I2C function after receiving an error, if retries remain, where portTICK_PERIOD_MS is a constant included from a FreeRTOS header */
    i2c_settings->i2c_max_error_message_length = 100; /* max length (char) of an I2C error message */
    i2c_settings->i2c_convert_pressure = 0x48; /* I2C command to convert pressure from analog to digital via ADC using OSR = 4096 */
    i2c_settings->i2c_convert_temperature = 0x58; /* I2C command to convert temperature from analog to digital via ADC using OSR = 4096 */
    i2c_settings->min_adc_wait_time = 8.61; /* minimum wait time (ms) for ADC conversion of pressure and temperature using OSR = 4096 */
    i2c_settings->i2c_read_adc = 0; /* I2C command to read uncompensated pressure or temperature data from ADC */
    i2c_settings->i2c_status_messages = false; /* determines whether status messages should be printed to standard output (error messages will always be printed to standard error) */
}

/*
Initialize I2C bus and controller.

Arguments:
    i2c_master_bus_handle_t *i2c_bus_handle: pointer to the I2C bus handle
    i2c_master_dev_handle_t *i2c_controller_handle: pointer to the I2C controller handle
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void init_i2c_bus_and_controller(i2c_master_bus_handle_t *i2c_bus_handle, i2c_master_dev_handle_t *i2c_controller_handle, I2CSettings *i2c_settings) {
    /* setup I2C bus */
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = i2c_settings->i2c_clock_source, /* default clock source*/
        .i2c_port = i2c_settings->i2c_port, /* I2C port number */
        .scl_io_num = i2c_settings->i2c_scl_gpio, /* GPIO for I2C SCL */
        .sda_io_num = i2c_settings->i2c_sda_gpio, /* GPIO for I2C SDA */
        .glitch_ignore_cnt = i2c_settings->glitch_ignore_count, /* glitch ignore period of master bus */
        .flags.enable_internal_pullup = i2c_settings->internal_pullup_resistor_state, /* state (enabled or disabled) of internal pullup resistor */
    };
    esp_err_t error_code = i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle);
    const char *error = esp_err_to_name(error_code);
    uint8_t retries = i2c_settings->i2c_max_retries;
    while (strcmp(error, "ESP_OK") != 0) {
        fprintf(stderr, "Error %s received while creating new I2C bus.\n", error);
        if (retries == 0) {
            fprintf(stderr, "Received errors after attempting to create new I2C bus %hhu times. Restarting...\n", i2c_settings->i2c_max_retries);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings->i2c_retry_delay);
        error_code = i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle);
        error = esp_err_to_name(error_code);
    }
    if (i2c_settings->i2c_status_messages) {
        printf("Created new I2C bus.\n");
    }

    /* setup I2C controller */
    i2c_device_config_t i2c_controller_config = {
        .dev_addr_length = i2c_settings->i2c_peripheral_address_bit_length, /* I2C address bit length (7) of peripheral, defined by I2C_ADDR_BIT_LEN_7 macro */
        .device_address = i2c_settings->peripheral_address, /* I2C address of peripheral */
        .scl_speed_hz = i2c_settings->scl_frequency, /* I2C SCL line frequency (kHz)*/
    };
    error_code = i2c_master_bus_add_device(*i2c_bus_handle, &i2c_controller_config, i2c_controller_handle);
    error = esp_err_to_name(error_code);
    retries = i2c_settings->i2c_max_retries;
    while (strcmp(error, "ESP_OK") != 0) {
        fprintf(stderr, "Error %s received while creating new I2C controller.\n", error);
        if (retries == 0) {
            fprintf(stderr, "Received errors after attempting to create new I2C controller %hhu times. Restarting...\n", i2c_settings->i2c_max_retries);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings->i2c_retry_delay);
        error_code = i2c_master_bus_add_device(*i2c_bus_handle, &i2c_controller_config, i2c_controller_handle);
        error = esp_err_to_name(error_code);
    }
    if (i2c_settings->i2c_status_messages) {
        printf("Created new I2C controller.\n");
    }
}

/*
Write command to peripheral.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint8_t command: command to send to peripheral
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void write_to_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint8_t command, I2CSettings *i2c_settings) {
    uint8_t write_buffer[1] = {command};
    if (i2c_settings->i2c_status_messages) {
        printf("Writing command %hhu to peripheral at address %hhu...\n", command, i2c_settings->peripheral_address);
    }
    esp_err_t error_code = i2c_master_transmit(i2c_controller_handle, write_buffer, sizeof(uint8_t), i2c_settings->i2c_controller_timeout);
    const char *error = esp_err_to_name(error_code);
    uint8_t retries = i2c_settings->i2c_max_retries;
    while (strcmp(error, "ESP_OK") != 0) {
        fprintf(stderr, "Error %s received when writing command %hhu to peripheral.\n", error, command);
        if (retries == 0) {
            fprintf(stderr, "Received errors after attempting to write to peripheral %hhu times. Restarting...\n", i2c_settings->i2c_max_retries);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings->i2c_retry_delay);
        error_code = i2c_master_transmit(i2c_controller_handle, write_buffer, sizeof(uint8_t), i2c_settings->i2c_controller_timeout);
        error = esp_err_to_name(error_code);
    }
    if (i2c_settings->i2c_status_messages) {
        printf("Wrote command %hhu to peripheral.\n", command);
    }
}

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
void read_from_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint8_t *read_buffer, uint8_t read_buffer_length, I2CSettings *i2c_settings) {
    if (i2c_settings->i2c_status_messages) {
        printf("Reading from peripheral...\n");
    }
    esp_err_t error_code = i2c_master_receive(i2c_controller_handle, read_buffer, sizeof(uint8_t) * read_buffer_length, i2c_settings->i2c_controller_timeout);
    const char *error = esp_err_to_name(error_code);
    uint8_t retries = i2c_settings->i2c_max_retries;
    while (strcmp(error, "ESP_OK") != 0) {
        fprintf(stderr, "Error %s received while reading from peripheral.\n", error);
        if (retries == 0) {
            fprintf(stderr, "Received errors after attempting to read from peripheral %hhu times. Restarting...\n", i2c_settings->i2c_max_retries);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings->i2c_retry_delay);
        error_code = i2c_master_receive(i2c_controller_handle, read_buffer, sizeof(uint8_t) * read_buffer_length, i2c_settings->i2c_controller_timeout);
        error = esp_err_to_name(error_code);
    }
    if (i2c_settings->i2c_status_messages) {
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
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (bool) true if an error was received when probing the I2C peripheral, false otherwise
*/
bool probe_peripheral(i2c_master_bus_handle_t i2c_bus_handle, I2CSettings *i2c_settings) {
    char *error = malloc(sizeof(char) * (i2c_settings->i2c_max_error_message_length + 1));
    bool error_received = false;
    if (i2c_settings->i2c_status_messages) {
        printf("Probing peripheral at address %hhu...\n", i2c_settings->peripheral_address);
    }
    esp_err_t error_code = i2c_master_probe(i2c_bus_handle, (uint16_t) i2c_settings->peripheral_address, i2c_settings->i2c_controller_timeout);
    strcpy(error, esp_err_to_name(error_code));
    if (strcmp(error, "ESP_OK") == 0) {
        if (i2c_settings->i2c_status_messages) {
            printf("Peripheral at address %hhu successfully probed with error status %s.\n", i2c_settings->peripheral_address, error);
        }
    } else {
        error_received = true;
        fprintf(stderr, "Unsuccessful probe of peripheral at address %hhu with error status %s.\n", i2c_settings->peripheral_address, error);
    }
    free(error);
    return error_received;
}

/*
Initialize peripheral. Index 0 of the returned array consists of, from MSB to LSB: a 4 bit cyclic redundancy check value, the 7 bits 0b0100100, and 5 bit factory settings. Indexes 1 to 6 of the returned array contain the peripheral's calibration coefficients.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint16_t *calibration_coefficients: array of calibration coefficients, CRC value, and factory settings
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void init_peripheral(i2c_master_dev_handle_t i2c_controller_handle, uint16_t *calibration_coefficients, I2CSettings *i2c_settings) {
    if (i2c_settings->i2c_status_messages) {
        printf("Initializing peripheral...\n");
    }
    write_to_peripheral(i2c_controller_handle, i2c_settings->i2c_peripheral_reset, i2c_settings); /* reset peripheral */
    for (uint8_t i = 0; i < 7; i++) { /* read calibration coefficients from PROM at PROM addresses 0b001 to 0b110 */
        uint8_t read_buffer[2];
        write_to_peripheral(i2c_controller_handle, (i2c_settings->i2c_peripheral_prom_read_prefix << 4) | (i << 1), i2c_settings); /* read PROM from peripheral at PROM address */
        read_from_peripheral(i2c_controller_handle, read_buffer, 2, i2c_settings);
        calibration_coefficients[i] = (read_buffer[0] << 8) | read_buffer[1];
    }
    uint8_t retries = i2c_settings->i2c_max_retries;
    uint8_t crc_expected = calibration_coefficients[0] >> 12;
    uint8_t crc_calculated = calc_crc(calibration_coefficients);
    while (crc_expected ^ crc_calculated) { /* compare received CRC value from PROM with calculated CRC value */
        fprintf(stderr, "Calculated CRC %hhu did not match expected CRC %hhu.\n", crc_calculated, crc_expected);
        if (retries == 0) {
            fprintf(stderr, "Calculated CRC did not match expected CRC %hhu times. Restarting...\n", i2c_settings->i2c_max_retries);
            esp_restart();
        }
        retries--;
        fprintf(stderr, "Retrying...\n");
        vTaskDelay(i2c_settings->i2c_retry_delay);
        write_to_peripheral(i2c_controller_handle, i2c_settings->i2c_peripheral_reset, i2c_settings);
        for (uint8_t i = 0; i < 7; i++) { /* read calibration coefficients from PROM at PROM addresses 0b001 to 0b110 */
            uint8_t read_buffer[2];
            write_to_peripheral(i2c_controller_handle, (i2c_settings->i2c_peripheral_prom_read_prefix << 4) | (i << 1), i2c_settings); /* read PROM from peripheral at PROM address */
            read_from_peripheral(i2c_controller_handle, read_buffer, 2, i2c_settings);
            calibration_coefficients[i] = (read_buffer[0] << 8) | read_buffer[1];
        }
        crc_expected = calibration_coefficients[0] >> 12;
        crc_calculated = calc_crc(calibration_coefficients);
    }
    if (i2c_settings->i2c_status_messages) {
        printf("CRC passed.\n");
        printf("Successfully initialized peripheral.\n");
    }
}

/*
Read uncompensated temperature and pressure from peripheral. Uncompensated temperature and pressure are first converted from analog to digital by ADC before being read.

Arguments:
    i2c_master_dev_handle_t i2c_controller_handle: handle for I2C controller
    uint32_t *uncompensated_temperature_and_pressure: array of size uint32_t and length 2 in which the uncompensated temperature and pressure values will be stored, with temperature at index 0 and pressure at index 1
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void read_uncompensated_temperature_and_pressure(i2c_master_dev_handle_t i2c_controller_handle, uint32_t *uncompensated_temperature_and_pressure, I2CSettings *i2c_settings) {
    if (i2c_settings->i2c_status_messages) {
        printf("Reading uncompensated temperature...\n");
    }
    write_to_peripheral(i2c_controller_handle, i2c_settings->i2c_convert_temperature, i2c_settings);
    vTaskDelay(i2c_settings->min_adc_wait_time);
    write_to_peripheral(i2c_controller_handle, i2c_settings->i2c_read_adc, i2c_settings);
    uint8_t read_buffer[3];
    read_from_peripheral(i2c_controller_handle, read_buffer, 3, i2c_settings);
    uint32_t temperature = 0;
    temperature = temperature | read_buffer[0];
    for (uint8_t i = 1; i < 3; i++) {
        temperature = temperature << 8;
        temperature = temperature | read_buffer[i];
    }
    if (i2c_settings->i2c_status_messages) {
        printf("Successfully read uncompensated temperature: %lu\n", temperature);
    }
    uncompensated_temperature_and_pressure[0] = temperature;
    if (i2c_settings->i2c_status_messages) {
        printf("Reading uncompensated pressure...\n");
    }
    write_to_peripheral(i2c_controller_handle, i2c_settings->i2c_convert_pressure, i2c_settings);
    vTaskDelay(i2c_settings->min_adc_wait_time);
    write_to_peripheral(i2c_controller_handle, i2c_settings->i2c_read_adc, i2c_settings);
    read_from_peripheral(i2c_controller_handle, read_buffer, 3, i2c_settings);
    uint32_t pressure = 0;
    pressure = pressure | read_buffer[0];
    for (uint8_t i = 1; i < 3; i++) {
        pressure = pressure << 8;
        pressure = pressure | read_buffer[i];
    }
    if (i2c_settings->i2c_status_messages) {
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
    I2CSettings *i2c_settings: pointer to an I2CSettings object initialized with the I2C settings
Returns:
    (void) nothing
*/
void calc_compensated_temperature_and_pressure(uint16_t *calibration_coefficients, uint32_t uncompensated_temperature, uint32_t uncompensated_pressure, double *compensated_temperature_and_pressure, I2CSettings *i2c_settings) {
    /* first order temperature and pressure calculations */

    /* calculate first order temperature */
    double reference_temperature = calibration_coefficients[5];
    double temperature_coefficient_of_temperature = calibration_coefficients[6];
    double temp_diff = uncompensated_temperature - reference_temperature * 0x100;
    double compensated_temperature = 2000 + (temp_diff * temperature_coefficient_of_temperature) / (double) 0x800000;
    if (i2c_settings->i2c_status_messages) {
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
    if (i2c_settings->i2c_status_messages) {
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
    if (i2c_settings->i2c_status_messages) {
        printf("Compensated temperature second order: %lf\n", compensated_temperature_c);
        printf("Compensated pressure second order: %lf\n", compensated_pressure_mbar);
    }
    compensated_temperature_and_pressure[0] = compensated_temperature_c;
    compensated_temperature_and_pressure[1] = compensated_pressure_mbar;
}

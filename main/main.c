#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "read_temperature_pressure.h"

/* task-specific parameters */
typedef struct task_parameters {
    I2CSettings *i2c_settings;
    i2c_master_dev_handle_t *i2c_controller_handle;
    uint16_t *calibration_coefficients;
} TaskParameters;

/*
Get compensated temperature and pressure from sensor for given I2C port and print to standard output. Meant to be run by a created task.

Arguments:
    void *x: pointer (cast to void *) to TaskParameters object initialized with the current task's I2C settings (including the task-specific I2C port number, I2C SCL GPIO port, and I2C SDA GPIO port), I2C controller handle, and calibration coefficients for the sensor on the current task's specified I2C port
Returns:
    (void) nothing
*/
void get_temperature_and_pressure_task(void *x) {
    TaskParameters *task_parameters = (TaskParameters *) x;
    I2CSettings *i2c_settings = task_parameters->i2c_settings;
    i2c_master_dev_handle_t *i2c_controller_handle = task_parameters->i2c_controller_handle;
    uint16_t *calibration_coefficients = task_parameters->calibration_coefficients;
    uint32_t uncompensated_temperature_and_pressure[2]; /* array containing the uncompensated temperature and pressure values */
    double compensated_temperature_and_pressure[2]; /* array containing the compensated temperature and pressure values */

    while (true) {
        read_uncompensated_temperature_and_pressure(*i2c_controller_handle, uncompensated_temperature_and_pressure, i2c_settings); /* read uncompensated temperature and pressure */
        calc_compensated_temperature_and_pressure(calibration_coefficients, uncompensated_temperature_and_pressure[0], uncompensated_temperature_and_pressure[1], compensated_temperature_and_pressure, i2c_settings); /* calculate the compensated temperature and pressure */
        printf("I2C port, compensated temperature (degrees C), compensated pressure (mbar): %hhu, %lf, %lf\n", i2c_settings->i2c_port, compensated_temperature_and_pressure[0], compensated_temperature_and_pressure[1]);
    }
}

void app_main(void) {
    const uint8_t NUM_I2C_PORTS = 1; /* number of I2C ports */
    const uint8_t i2c_scl_gpio[2] = { /* I2C SCL GPIO pins, one for each I2C port */
        19,
        22,
    };
    const uint8_t i2c_sda_gpio[2] = { /* I2C SDA GPIO pins, one for each I2C port */
        21,
        23,
    };
    if (NUM_I2C_PORTS == 1) { /* use only one task */
        I2CSettings i2c_settings; /* I2C settings object initialized with the I2C settings */
        i2c_master_bus_handle_t i2c_bus_handle; /* handle for I2C bus */
        i2c_master_dev_handle_t i2c_controller_handle; /* handle for I2C controller */
        uint16_t calibration_coefficients[7]; /* calibration coefficients, CRC value, and factory settings */
        uint32_t uncompensated_temperature_and_pressure[2]; /* array containing the uncompensated temperature and pressure values */
        double compensated_temperature_and_pressure[2]; /* array containing the compensated temperature and pressure values */

        init_i2c_settings(&i2c_settings, 0, i2c_scl_gpio[0], i2c_sda_gpio[0]); /* initialize I2C settings */
        init_i2c_bus_and_controller(&i2c_bus_handle, &i2c_controller_handle, &i2c_settings); /* initialize I2C bus and controller */
        init_peripheral(i2c_controller_handle, calibration_coefficients, &i2c_settings); /* initialize peripheral and get calibration coefficients, CRC value, and factory settings */
        while (true) {
            read_uncompensated_temperature_and_pressure(i2c_controller_handle, uncompensated_temperature_and_pressure, &i2c_settings); /* read uncompensated temperature and pressure */
            calc_compensated_temperature_and_pressure(calibration_coefficients, uncompensated_temperature_and_pressure[0], uncompensated_temperature_and_pressure[1], compensated_temperature_and_pressure, &i2c_settings); /* calculate the compensated temperature and pressure */
            printf("compensated temperature (degrees C), compensated pressure (mbar): %lf, %lf\n", compensated_temperature_and_pressure[0], compensated_temperature_and_pressure[1]);
        }
    } else { /* create two tasks, one per core */
        I2CSettings i2c_settings[2]; /* array of I2CSettings objects, one for each core, initialized with the I2C settings specific to that core */
        i2c_master_bus_handle_t i2c_bus_handles[2]; /* array of handles for I2C bus, one for each core */
        i2c_master_dev_handle_t i2c_controller_handles[2]; /* array of handles for I2C controller, one for each core */
        uint16_t calibration_coefficients[2][7]; /* array of arrays, one per core, of calibration coefficients, CRC value, and factory settings */
        TaskParameters task_parameters[2]; /* array of TaskParameter objects containing the parameters passed to each task (one task per core) */
        const uint16_t TASK_STACK_SIZE = 4096; /* the stack size allocated to each task */
        const uint8_t TASK_PRIORITY = 1; /* the priority of each task, set to highest priority on each core */

        /* create one task for each core to read and process data from an I2C port */
        for (uint8_t i = 0; i < NUM_I2C_PORTS; i++) {
            init_i2c_settings(&i2c_settings[i], i, i2c_scl_gpio[i], i2c_sda_gpio[i]); /* initialize I2C settings */
            init_i2c_bus_and_controller(&i2c_bus_handles[i], &i2c_controller_handles[i], &i2c_settings[i]); /* initialize I2C bus and controller */
            init_peripheral(i2c_controller_handles[i], calibration_coefficients[i], &i2c_settings[i]); /* initialize peripheral and get calibration coefficients, CRC value, and factory settings */
            char s[2];
            sprintf(s, "%d", i);
            char task_name[7] = "core_"; /* task_name is core_0 for task on core 0 or core_1 for task on core 1 */
            strcat(task_name, s);
            task_parameters[i].i2c_settings = &i2c_settings[i];
            task_parameters[i].i2c_controller_handle = &i2c_controller_handles[i];
            task_parameters[i].calibration_coefficients = calibration_coefficients[i];
            xTaskCreatePinnedToCore(get_temperature_and_pressure_task, task_name, TASK_STACK_SIZE, (void *) &task_parameters[i], TASK_PRIORITY, NULL, i);
        }
        /* delay main task indefinitely to keep alive */
        while (true) {
            vTaskDelay(1000);
        }
    }
}

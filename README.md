# Temperature and Pressure Sensor Firmware and Utility

This software reads temperature and pressure from a MS5837 or MS5839 sensor via I2C with an ESP32 microcontroller and outputs the data over serial.

Though many of the features of this software can be used on [MacOS](#macos) or [Windows](#windows), this software has primarily been tested on GNU/Linux systems and this guide is oriented primarily toward these systems.

## Hardware Requirements

This software requires the following hardware to run:

* **A computer**. This software was tested on a [Raspberry Pi 4](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf) (ARM64) running [Raspberry Pi OS](https://www.raspberrypi.com/software/operating-systems/) (based on [Debian 12](https://www.debian.org/releases/bookworm/)), a x86-64 (AMD64) computer running [Debian 13](https://www.debian.org/releases/trixie/), and a x86-64 (AMD64) computer running Windows 11.

* **An ESP32 microcontroller.** This software was tested on an [Espressif ESP32-WROOM-32](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.pdf) microcontroller (MCU) mounted on an [Inland ESP32](https://community.microcenter.com/kb/articles/652-inland-esp32-core-board-black-and-eco-friendly) development board and an [Espressif ESP32-WROOM-32E](https://documentation.espressif.com/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf) MCU mounted on an [Espressif ESP32 DevKitC](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/index.html) development board.

* **A MS5837 or MS5839 temperature and pressure sensor**. This software was tested on [TE Connectivity MS5837-02BA](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5837-02BA01&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20000979-00) and [TE Connectivity MS5839-02BA](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5839-02BA&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20008669-50) temperature and pressure sensors, with the MS5839 sensor mounted on a [Mikroe Pressure 22 Click](https://www.mikroe.com/pressure-22-click?srsltid=AfmBOoqSizi4NoiV8QF_MCegA7pMt5wmre6OOaNLoXm300nrrdypXcyi) development board.

## Hardware Configuration

The MS5839 sensor is the default sensor used in `main/main.c`. To use a MS5837 sensor, set the `SENSOR_MODEL` array in the `app_main` function of `main/main.c` to `"MS5837"`.

Connect the SCL and SDA lines of the sensor to GPIO pins 19 and 21 of the ESP32, respectively. The default GPIO pins for the SCL and SDA lines can be changed by modifying the `i2c_scl_gpio` and `i2c_sda_gpio` arrays in the `app_main` function of `main/main.c`, by passing the desired values to the `init_i2c_settings` function of `read_temperature_pressure/read_temperature_pressure.c` when initializing an `I2CSettings` object in `main/main.c`, or by creating an `I2CSettings` object directly (without using `init_i2c_settings`) with the desired values. See [Support for Using Multiple Sensors Simultaneously](#support-for-using-multiple-sensors-simultaneously) for more information on configuration of SCL and SDA GPIO pins when using two sensors.

The default I2C settings are set in the definition of the `I2CSettings` object in the `init_i2c_settings` function of `read_temperature_pressure/read_temperature_pressure.c`. These settings, which include the I2C SCL line frequency and minimum ADC wait time, can be changed as needed by passing the desired values to the `init_i2c_settings` function of `read_temperature_pressure/read_temperature_pressure.c` when initializing an `I2CSettings` object in `main/main.c`.

Consult the application circuit provided in the [MS5837](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5837-02BA01&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20000979-00) or [MS5839](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5839-02BA&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20008669-50) sensor datasheets. The circuit contains two 10k&#x03a9; pull-up resistors on the SCL and SDA lines and a 100nF capacitor connected to VDD and GND of the sensor. If using a MS5839 sensor mounted to a Mikroe Pressure 22 Click development board, note that a 470nF capacitor is connected to VDD and GND of the sensor (see [schematic](https://download.mikroe.com/documents/add-on-boards/click/pressure_22_click/Pressure_22_click_v100_Schematic.PDF)).

### Pull-Up Resistance Calculation

I2C communication between the sensor and the MCU requires 2 pull-up resistors on the SCL and SDA lines. Calculate the minimum and maximum resistance needed for the pull-up resistors using the equations described in this Texas Instruments [technical paper](https://www.ti.com/lit/an/slva689/slva689.pdf?ts=1764607698210).

The `utility` directory contains a Python module `calculate_pull_up_resistance` which can be used to calculate the minimum and maximum pull-up resistance required according to the following equations.

#### Minimum Pull-Up Resistance Calculation

The minimum pull-up resistance is:

```math
R_{min} = \dfrac{V_{CC} - V_{OL\_max}}{I_{OL}}
```

where $`R_{min}`$ is the minimum pull-up resistance (&#x03a9;), $`V_{CC}`$ is the power supply voltage (V), $`V_{OL\_max}`$ is the maximum low-level output voltage (V), and $`I_{OL}`$ is the low-level output (sink) current (A).

For example, using $`V_{CC} = 3.3\text{V}`$, $`V_{OL\_max} = 0.1 \times 3.3\text{V} = 0.33\text{V}`$, and $`I_{OL} = 0.028\text{A}`$, $`R_{min} = 106\Omega`$.

#### Maximum Pull-Up Resistance Calculation

The maximum pull-up resistance is:

```math
R_{max} = \dfrac{t_r}{c \times \ln\left(\dfrac{V_{CC} - V_{IL\_max}}{V_{CC} - V_{IH\_min}}\right)}
```

where $`R_{max}`$ is the maximum pull-up resistance (&#x03a9;), $`t_r`$ is the I2C rise time from maximum low-level input voltage to minimum high-level input voltage (s), $`c`$ is the bus capacitace for each I2C line (F), $`V_{CC}`$ is the power supply voltage (V), $`V_{IL\_max}`$ is the maximum low-level input voltage (V), and $`V_{IH\_min}`$ is the minimum high-level input voltage (V).

For example, using $`t_r = 1 \times 10^{-6}\text{s}`$, $`c = 50 \times 10^{-12}\text{F}`$, $`V_{CC} = 3.3\text{V}`$, $`V_{IL\_max} = 0.25 \times 3.3\text{V} = 0.825\text{V}`$, and $`V_{IH\_min} = 0.75 \times 3.3\text{V} = 2.5\text{V}`$, $`R_{max} = 18\text{k}\Omega`$.

## Software Installation

### esp-idf Installation

`esp-idf` is required to build the program, upload it to the ESP32, and monitor the serial output. For information on installing `esp-idf`, see the [official documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) and [this guide](https://www.jimmyvanhout.com/docs/run_program_on_esp32/).

### Firmware and Utility Installation

To install the temperature and pressure sensor firmware and utility programs, clone the repository from GitHub:

```bash
git clone git@github.com:JimmyVanHout/temperature_pressure_sensor.git
```

The firmware and utility programs will be located in the `temperature_pressure_sensor` directory. Some of the files within this directory include the following:

```
CMakeLists
    root
        CMakeLists.txt
    main
        CMakeLists.txt
    read_temperature_pressure
        CMakeLists.txt
main
    main.c
read_temperature_pressure
    read_temperature_pressure.h
    read_temperature_pressure.c
```

## Formatting the Project Structure for esp-idf

To format the project structure required for `esp-idf`, change to the project directory (cloned from GitHub) if necessary and then run the `format.py` Python utility located in the `utility` directory:

```bash
cd temperature_pressure_sensor
python3 utility/format.py
```

The `format.py` program will create a new subdirectory called `temperature_pressure_sensor` within the project directory (note that the project directory is *also* named `temperature_pressure_sensor`) containing the proper project structure required by `esp-idf`. For example, the default file structure within the newly created `temperature_pressure_sensor` subdirectory after running `format.py` is:

```
CMakeLists.txt                              # copied from CMakeLists/root/CMakeLists.txt

components
    read_temperature_pressure
        CMakeLists.txt                      # copied from CMakeLists/read_temperature_pressure/CMakeLists.txt
        read_temperature_pressure.c         # copied from read_temperature_pressure/read_temperature_pressure.c
        include
            read_temperature_pressure.h     # copied from read_temperature_pressure/read_temperature_pressure.h
main
    CMakeLists.txt                          # copied from CMakeLists/root/CMakeLists.txt
    main.c                                  # copied from main/main.c
```

The `format.py` program takes the following command-line options:

`-d <path_to_directory>` or `--dir <path_to_directory>`: Specify the location of the project directory (by default named `temperature_pressure_sensor` when cloned from GitHub), where `<path_to_directory>` is the path to the project directory. If `-d` or `--dir` is not specified, then the current directory is assumed to be the project directory, unless the program is run from within the `utility` subdirectory in which case the `format.py` program will change to the parent directory (the project directory) automatically for convenience. The `-d` or `--dir` options are useful for running the `format.py` command from another directory outside of the project directory.

`-c` or `--component-only`: Only create the `read_temperature_pressure` component and the necessary `CMakeLists.txt` files, do not create `main`, `main/main.c`, or `main/CMakeLists.txt`. This is useful for incorporating the `read_temperature_pressure` component into an existing project. The `-c` and `--component-only` options cannot be used with the `-m` or `--main` options.

`-m <path_to_main>` or `--main <path_to_main>`: Specify the location of the main source file to be compiled and executed. The default is `main/main.c`. This is useful for building, uploading, and monitoring unit tests. For example, the `-m` or `--main` flag can be used to copy a unit test file in the project directory to the `main` directory in the `temperature_pressure_sensor` directory created by `format.py` so that the unit test will be compiled and executed as the main executable on the ESP32 after the build and upload process. The `-m` and `--main` options cannot be used with the `-c` or `--component-only` options.

## Building, Uploading, and Monitoring the Program

To build, upload, and monitor the program:

1. Change to the program directory:

    ```bash
    cd <program_directory_path>
    ```

    where `<program_directory_path>` is the path to the *subdirectory*, `temperature_pressure_sensor`, that was created by running `python3 format.py` (again note that this is different from the *project directory* that was cloned from GitHub).

1. Build the program:

    ```bash
    idf.py build
    ```

1. Upload the program to the ESP32:

    ```bash
    idf.py -p <port> -b <baud_rate> flash
    ```

    where `<port>` is an optional argument specifying the port on which the ESP32 is connected and `baud_rate` is an optional argument specifying the baud rate. See the [documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/establish-serial-connection.html#connect-esp32-to-pc) and [this guide](https://www.jimmyvanhout.com/docs/run_program_on_esp32/#build-flash-and-monitor-the-program) for more information.

1. Monitor the output of the program over serial:

    ```bash
    idf.py -p <port> monitor
    ```

    To exit the monitor, press `Ctrl + ]`.

For additional information on building, uploading, and monitoring the program using `esp-idf`, see the [official documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) and [this guide](https://www.jimmyvanhout.com/docs/run_program_on_esp32/).

## Support for Using Multiple Sensors Simultaneously

By default, the firmware supports the use of one MS5837 or MS5839 sensor. The firmware also supports the use of two sensors of the same type simultaneously using multithreading, enabling the use of one task on each of the ESP32's two cores. Each task uses one of the two I2C ports available on the ESP32, allowing for higher throughput. To enable support for multiple sensors simultaneously, set the desired number of I2C ports by setting `NUM_I2C_PORTS` to `2` in the `app_main` function of `main/main.c`.

The default SCL and SDA GPIO pins for the first sensor are 19 and 21, respectively, and the default SCL and SDA GPIO pins for the second sensor are 22 and 23, respectively. These can be changed by modifying the `i2c_scl_gpio` and `i2c_sda_gpio` arrays in the `app_main` function of `main/main.c`, by passing the desired values to the `init_i2c_settings` function of `read_temperature_pressure/read_temperature_pressure.c` when initializing an `I2CSettings` object for each task in `main/main.c`, or by creating an `I2CSettings` object directly (without using `init_i2c_settings`) with the desired values for each task.

## Logging

Temperature and pressure data can be read from the serial output of the microcontroller and logged to a CSV file and SQLite database by running the `utility/log.py` program on the connected computer.

### Dependencies

First, change to the `utility` directory within the project directory. Then, create the virtual environment (if necessary) and activate it. Finally, install the required dependencies:

```bash
cd <project_directory>/utility
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Usage

Run:

```bash
python3 utility/log.py -d <dir_path> -m <mcu_id> -p <port> -b <baud_rate>
```

The `log.py` utility takes the following command-line options, most of which are required:

`-d <dir_path>` or `--dir <dir_path>`: (optional) an option specifying the path `<dir_path>` to the directory to change to before executing the program. If you specify a directory using this option, the `data` directory containing the data log and database will be saved there. Otherwise, the `data` directory will be saved in the project directory by default.

`-m <mcu_id>` or `--mcu-id <mcu_id>`: (required) an option specifying an identifier `<mcu_id>` for the microcontroller from which data will be logged. This identifier must be either 0 or 1. If only one microcontroller is used, it must be assigned the identifier 0.

`-p <port>` or `--port <port>`: (required) an option specifying the port `<port>` on which the microcontroller is connected.

`-b <baud_rate>` or `--baud-rate <baud_rate>`: (required) an option specifying the baud rate `<baud_rate>`.

Serial data from up to two microcontrollers (each with a different identifier and each using up to two I2C ports) can be logged simultaneously using two processes on a connected computer. Each microcontroller's output is logged to the same database but separate CSV files.

The CSV file will be named `data_mcu_id_<mcu_id>.csv` for each microcontroller identifier and will be saved in a directory named `data` along with the database `data.db`. The default location for the `data` directory is the project directory, but this can be changed using the `-d` option as noted above.

### Examples

For example, to log data from one microcontroller (using up to two sensors, each on a separate I2C port) using port `/dev/ttyUSB0` and baud rate `115200`, run:

```bash
python3 utility/log.py -m 0 -p /dev/ttyUSB0 -b 115200
```

As another example, to log data from two microcontrollers (using up to two sensors each), with one microcontroller using port `/dev/ttyUSB0` and baud rate `115200` and the other microcontroller using port `/dev/ttyUSB1` and baud rate `115200`, in one terminal run:

```bash
python3 utility/log.py -m 0 -p /dev/ttyUSB0 -b 115200
```

and in another terminal run:

```bash
python3 utility/log.py -m 1 -p /dev/ttyUSB1 -b 115200
```

## Set Up and Use on Non-GNU/Linux Systems

### MacOS

Aside from the installation of ESP-IDF, most of this guide is applicable to MacOS. See the [official documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/macos-setup.html) for installation of ESP-IDF on MacOS.

### Windows

See the [official documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html) for installation of ESP-IDF on Windows 11, and please see this [guide](https://gist.github.com/JimmyVanHout/6a982df871cf4748bda9aba4fa062c03) for overall setup and use of this software on Windows 11.

## Support

Please open an [Issue](https://github.com/JimmyVanHout/temperature_pressure_sensor/issues) for support.

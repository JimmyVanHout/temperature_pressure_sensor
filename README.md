# Temperature and Pressure Sensor

This program reads temperature and pressure from a MS5839-02BA sensor via I2C with an ESP32 microcontroller and outputs the data over serial.

## Hardware Requirements

This program requires the following hardware to run:

* **A computer**. This program was tested on a [Raspberry Pi 4](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf) (ARM64) running [Raspberry Pi OS](https://www.raspberrypi.com/software/operating-systems/) based on [Debian 12](https://www.debian.org/releases/bookworm/).

* **An ESP32 microcontroller.** This program was tested on a [ESP32-WROOM-32](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.pdf) microcontroller (MCU) on an [Inland ESP32](https://community.microcenter.com/kb/articles/652-inland-esp32-core-board-black-and-eco-friendly) development board.

* **A MS5839-02BA temperature and pressure sensor**. This program was tested on a [TE Connectivity MS5839-02BA](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5839-02BA&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20008669-50) temperature and pressure sensor on a [Mikroe Pressure 22 Click](https://www.mikroe.com/pressure-22-click?srsltid=AfmBOoqSizi4NoiV8QF_MCegA7pMt5wmre6OOaNLoXm300nrrdypXcyi) development board.

## Hardware Configuration

Connect the SCL and SDA lines of the MS5839-02BA sensor to GPIO pins 19 and 21 of the ESP32, respectively. The default GPIO pins for the SCL and SDA lines can be changed by passing the desired values to `init_i2c_settings` in `read_temperature_pressure/read_temperature_pressure.c` when initializing an `I2CSettings` object. See [Support for Multiple Sensors](#support-for-multiple-sensors) for more information on configuration of SCL and SDA GPIO pins when using two sensors.

Consult the application circuit provided in the MS5839-02BA [datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5839-02BA&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20008669-50). The circuit contains two 10k&#x03a9; pull-up resistors on the SCL and SDA lines and a 100nF capacitor connected to VDD and GND of the sensor. On the Pressure 22 Click development board, 2 10k&#x03a9; resistors are used on the SCL and SDA lines and a 470nF capacitor is connected to VDD and GND of the sensor (see [schematic](https://download.mikroe.com/documents/add-on-boards/click/pressure_22_click/Pressure_22_click_v100_Schematic.PDF)).

### Pull-Up Resistance Calculation

I2C communication between the sensor and the MCU requires 2 pull-up resistors on the SCL and SDA lines. Calculate the minimum and maximum resistance needed for the pull-up resistors using the equations described in this Texas Instruments [technical paper](https://www.ti.com/lit/an/slva689/slva689.pdf?ts=1764607698210).

The `utility` directory contains a Python module `calculate_pull_up_resistance` which can be used to calculate the minimum and maximum pull-up resistance required according to the following equations.

#### Minimum Pull-Up Resistance Calculation

The minimum pull-up resistance is:

```math
R_{min} = \dfrac{V_{CC} - V_{OL\_max}}{I_{OL}}
```

where $`R_{min}`$ is the minimum pull-up resistance (&#x03a9;), $`V_{CC}`$ is the power supply voltage (V), $`V_{OL\_max}`$ is the maximum low-level output voltage (V), and $`I_{OL}`$ is the low-level output (sink) current.

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

### Temperature and Pressure Program Installation

To install the temperature and pressure sensor program and related files, clone the repository from GitHub:

    ```bash
    git clone git@github.com:JimmyVanHout/temperature_pressure_sensor.git
    ```

The program and related files will be located in the `temperature_pressure_sensor` directory. Some of the files within this directory include the following:

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

The `format.py` program will create a new subdirectory `temperature_pressure_sensor` within the project directory (which is also named `temperature_pressure_sensor`) containing the proper project structure required by `esp-idf`. For example, the default file structure within the newly created `temperature_pressure_sensor` subdirectory after running `format.py` is:

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

`-d <path_to_directory>` or `--dir <path_to_directory>`: Specify the location of the project directory (by default named `temperature_pressure_sensor` when cloned from GitHub), where `<path_to_directory>` is the path to the project directory. If `-d` or `--dir` is not specified, then the current directory is assumed to be the project directory, unless the program is run from within the `utility` subdirectory in which case the `format.py` program will change to the parent (the project directory) automatically for convenience. The `-d` or `--dir` options are useful for running the `format.py` command from another directory outside of the project directory.

`-c` or `--component-only`: Only create the `read_temperature_pressure` component and the necessary `CMakeLists.txt` files, do not create `main`, `main/main.c`, or `main/CMakeLists.txt`. This is useful for incorporating the `read_temperature_pressure` component into an existing project. The `-c` and `--component-only` options cannot be used with the `-m` or `--main` options.

`-m <path_to_main>` or `--main <path_to_main>`: Specify the location of the main source file to be compiled and executed. The default is `main/main.c`. This is useful for building, uploading, and monitoring unit tests. For example, the `-m` or `--main` flag can be used to copy a unit test file `tests/main/test_main.c` to `main` so that it will be compiled and executed as the main executable on the ESP32 after the build and upload process. The `-m` and `--main` options cannot be used with the `-c` or `--component-only` options.

## Building, Uploading, and Monitoring the Program

For further information on building, uploading, and monitoring the program using `esp-idf`, see the [official documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) and [this guide](https://www.jimmyvanhout.com/docs/run_program_on_esp32/).

To build, upload, and monitor the program:

1. Change to the program directory:

    ```bash
    cd <program_directory_path>
    ```

    where `<program_directory_path>` is the path to the *subdirectory*, `temperature_pressure_sensor`, that was created by running `python format.py` (note that this is different from the *project directory* that was cloned from GitHub).

1. Build the program:

    ```bash
    idf.py build
    ```

1. Upload the program to the ESP32:

    ```bash
    idf.py -p <port> -b <baud_rate> flash
    ```

    where `<port>` is an optional argument specifying the port on which the ESP32 is connected and `baud_rate` is an optional argument specifying the baud rate. See the [documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/establish-serial-connection.html#connect-esp32-to-pc) for more information.

1. Monitor the output of the program over serial:

    ```bash
    idf.py -p <port> monitor
    ```

    To exit the monitor, press `Ctrl + ]`.

## Support for Multiple Sensors

By default, the program supports the use of one MS5839-02BA sensor. The program also supports the use of two MS5839-02BA sensors simultaneously using multithreading, enabling the use of one task on each of the ESP32's two cores. Each task uses one of the two I2C ports available on the ESP32, ensuring high throughput. To enable support for multiple sensors simultaneously, set the desired number of I2C ports by setting `NUM_I2C_PORTS` to `2` in the `app_main` function in `main/main.c`.

The default SCL and SDA GPIO pins for the first sensor are 19 and 21, respectively, and the default SCL and SDA GPIO pins for the second sensor are 22 and 23, respectively. These can be changed by modifying the `i2c_scl_gpio` and `i2c_sda_gpio` arrays in the `app_main` function in `main/main.c` or by passing the desired values to `init_i2c_settings` in `read_temperature_pressure/read_temperature_pressure.c` when initializing an `I2CSettings` object for each task.

## Support

Please open an [Issue](https://github.com/JimmyVanHout/temperature_pressure_sensor/issues) for support.

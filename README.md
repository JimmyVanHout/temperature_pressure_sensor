# Temperature and Pressure Sensor

This program reads temperature and pressure from a MS5839-02BA sensor via I2C with an ESP32 microcontroller and outputs the data over serial.

## Hardware Requirements

This program requires the following hardware to run:

* **A computer**. This program was tested on a [Raspberry Pi 4](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-datasheet.pdf) (ARM64) running [Raspberry Pi OS](https://www.raspberrypi.com/software/operating-systems/) based on [Debian 12](https://www.debian.org/releases/bookworm/).

* **An ESP32 microcontroller.** This program was tested on a [ESP32-WROOM-32](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.pdf) microcontroller (MCU) on an [Inland ESP32](https://community.microcenter.com/kb/articles/652-inland-esp32-core-board-black-and-eco-friendly) development board.

* **A MS5839-02BA temperature and pressure sensor**. This program was tested on a [TE Connectivity MS5839-02BA](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5839-02BA&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20008669-50) temperature and pressure sensor on a [Mikroe Pressure 22 Click](https://www.mikroe.com/pressure-22-click?srsltid=AfmBOoqSizi4NoiV8QF_MCegA7pMt5wmre6OOaNLoXm300nrrdypXcyi) development board.

## Hardware Configuration

Connect the SCL and SDA lines of the MS5839-02BA sensor to GPIO 32 and 33 of the ESP32, respectively. The GPIO ports for the SCL and SDA lines can be changed by changing the values of `I2CSettings i2c_settings.I2C_SCL_GPIO` and `I2CSettings i2c_settings.I2C_SDA_GPIO` in `main/main.c`, respectively.

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

### Building, Uploading, and Monitoring the Program

For more information on building, uploading, and monitoring the program using `esp-idf`, see the [official documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) and [this guide](https://www.jimmyvanhout.com/docs/run_program_on_esp32/).

To install the temperature and pressure sensor program:

1. Clone the repository from GitHub and change into the `temperature_pressure_sensor` directory:

    ```bash
    git clone git@github.com:JimmyVanHout/temperature_pressure_sensor.git
    cd temperature_pressure_sensor
    ```

1. Build the program:

    ```bash
    idf.py build
    ```

    > **Note:** If you receive an error `error: "LOG_LOCAL_LEVEL" redefined [-Werror]`, you will need to disable errors for default warnings by running `idf.py menuconfig` and selecting `Compiler options->Disable errors for default warnings`. This appears to be a bug in `esp-idf`.

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

## Support

Please open an [Issue](https://github.com/JimmyVanHout/temperature_pressure_sensor/issues) for support.

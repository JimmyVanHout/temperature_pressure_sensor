"""
# Temperature and Pressure Data Logger

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

Up to two microcontrollers (each with a different identifier) can be used simultaneously, with each using up to two I2C ports. Each microcontroller's output is logged to the same database but separate CSV files.

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
"""

import argparse
import contextlib
import csv
import datetime
import os
import re
import serial
import sqlite3
import webbrowser

def read_arguments():
    """
    Read arguments from the command line.

    Arguments:
        none

    Return:
        a dictionary containing a key-value mapping of command-line options to command-line arguments
    """
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("-d", "--dir")
    argument_parser.add_argument("-m", "--mcu-id", required=True)
    argument_parser.add_argument("-p", "--port", required=True)
    argument_parser.add_argument("-b", "--baud-rate", required=True)
    arguments = {key.replace("_", "-"): value for key, value in vars(argument_parser.parse_args()).items()}
    return arguments

def read_line_from_serial(serial_connection):
    """
    Read a line of output from the serial connection.

    If the line contains the microcontroller ID, I2C port, compensated temperature, and compensated pressure, all in the expected format, then the data is returned as a dictionary.

    Arguments:
        serial_connection: a serial.Serial object representing the serial connection

    Return:
        a dictionary containing the data, or None if the line read from serial did not contain data in the expected format
    """
    line = serial_connection.readline()
    pattern = re.compile(r"MCU ID, I2C port, compensated temperature \(degrees C\), compensated pressure \(mbar\): (?P<mcu_id>\d+), (?P<i2c_port>\d+), (?P<compensated_temperature>\d+\.\d+), (?P<compensated_pressure>\d+\.\d+)")
    match = pattern.match(line.decode())
    data = None
    if match:
        data = match.groupdict()
        data["date_time"] = datetime.datetime.now(tz=datetime.timezone.utc)
    return data

def data_to_str(data, borders=False):
    """
    Get a string representation of the serial data.

    Arguments:
        data: a dictionary containing the serial data, with keys date_time, mcu_id, i2c_port, compensated_temperature, and compensated_pressure
        borders: a boolean indicating whether top and bottom borders should be used to separate the data from other data when printed

    Return:
        a string representation of the serial data
    """
    s = [
        f"date-time: {data["date_time"]}",
        f"MCU ID: {data["mcu_id"]}",
        f"I2C port: {data["i2c_port"]}",
        f"compensated temperature: {data["compensated_temperature"]}",
        f"compensated pressure: {data["compensated_pressure"]}",
    ]
    if borders:
        t = "-" * 20
        s.insert(0, t)
        s.append(t)
    s = "\n".join(s)
    return s

def init_db(db_connection):
    """
    Initialize the database.

    Arguments:
        db_connection: the sqlite3 database connection object

    Return:
        the sqlite3 database connection cursor
    """
    cursor = db_connection.cursor()
    cursor.execute(
        """
        CREATE TABLE IF NOT EXISTS serial_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            date_time TEXT,
            mcu_id INTEGER,
            i2c_port INTEGER,
            compensated_temperature REAL,
            compensated_pressure REAL
        );
        """
    )
    db_connection.commit()
    return cursor

def log_data_to_db(db_connection, db_cursor, data):
    """
    Log serial data to a SQLite database.

    Arguments:
        db_connection: the database connection object returned by sqlite3.connect
        db_cursor: the database cursor object returned by sqlite3.cursor
        data: a dictionary containing the serial data
    """
    db_cursor.execute(
        """
        INSERT INTO serial_data (date_time, mcu_id, i2c_port, compensated_temperature, compensated_pressure) VALUES (?, ?, ?, ?, ?);
        """,
        (
            data["date_time"],
            data["mcu_id"],
            data["i2c_port"],
            data["compensated_temperature"],
            data["compensated_pressure"],
        )
    )
    db_connection.commit()

def init_csv(csv_file_path):
    """
    Initialize a CSV with the proper header if it does not exist.

    Arguments:
        csv_file_path: the path to the CSV file

    Return:
        nothing
    """
    # create file if it does not exist
    if not os.path.isfile(csv_file_path):
        file = open(csv_file_path, "w")
        file.close()

    # add header to file if it is not present
    with open(csv_file_path, "r+") as csv_file:
        csv_reader = csv.reader(csv_file)
        header = [
            "date_time",
            "mcu_id",
            "i2c_port",
            "compensated_temperature",
            "compensated_pressure",
        ]
        row = None
        try:
            row = next(csv_reader)
        except StopIteration:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(header)
        else:
            if row != header:
                raise Exception("CSV file exists with invalid header")

def log_data_to_csv(csv_writer, data):
    """
    Log serial data to a CSV file.

    Arguments:
        csv_writer: the CSV writer object returned by csv.writer
        data: a dictionary containing the serial data

    Return:
        nothing
    """
    row = [
        data["date_time"],
        data["mcu_id"],
        data["i2c_port"],
        data["compensated_temperature"],
        data["compensated_pressure"],
    ]
    csv_writer.writerow(row)

if __name__ == "__main__":
    # read arguments
    arguments = read_arguments()
    directory_path = arguments["dir"]
    mcu_id = arguments["mcu-id"]
    port = arguments["port"]
    baud_rate = arguments["baud-rate"]

    # change directory if necessary
    if directory_path:
        os.chdir(directory_path)
    elif "utility" == os.path.split(os.getcwd())[1]:
        os.chdir("..")
    
    # read and process data from serial
    os.makedirs("data", exist_ok=True) # create data directory
    csv_file_path = os.path.join("data", f"data_mcu_id_{mcu_id}.csv")
    db_file_path = os.path.join("data", "data.db")
    init_csv(csv_file_path) # initialize CSV file with proper header if it does not exist
    with (
        serial.Serial(port=port, baudrate=baud_rate) as serial_connection,
        contextlib.closing(sqlite3.connect(db_file_path)) as db_connection,
        open(csv_file_path, "a+") as csv_file,
    ):
        db_cursor = init_db(db_connection)
        csv_writer = csv.writer(csv_file)
        print(f"Connected to port {serial_connection.name}")
        while True:
            data = read_line_from_serial(serial_connection)
            if data and data["mcu_id"] == mcu_id:
                log_data_to_db(db_connection, db_cursor, data)
                log_data_to_csv(csv_writer, data)
                print(data_to_str(data, borders=True))

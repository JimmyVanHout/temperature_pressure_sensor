"""
# Temperature and Pressure Data Graphing Utility

Temperature and pressure data can be graphed in a web browser after it has been logged using `log.py`.

## Dependencies

First, change to the `utility` directory within the project directory. Then, create the virtual environment (if necessary) and activate it. Finally, install the required dependencies:

```bash
cd <project_directory>/utility
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

Run:

```python
python3 graph.py -p <port>
```

The command accepts the following arguments:

`-p` or `--port`: (optional) specify the port on which the web browser should listen on `localhost` (default is `8000`).

A web browser will be launched listening to `localhost` on port `8000` by default. This port can be changed using the `-p` option.
"""

import argparse
import contextlib
import flask
import os
import sqlite3
import webbrowser

DEFAULT_NUM_RECORDS = 40
DEFAULT_PORT = 8000
STATIC_FILE_DIRECTORY = "www"

os.chdir(os.path.dirname(os.path.abspath(__file__))) # change to directory of current file

# setup Flask app
app = flask.Flask(__name__, static_url_path="", static_folder=STATIC_FILE_DIRECTORY, template_folder=STATIC_FILE_DIRECTORY)

DB_FILE_PATH = os.path.join(os.pardir, "data", "data.db") # database file path

def read_arguments():
    """
    Read arguments from the command line.

    Arguments:
        none

    Return:
        a dictionary containing a key-value mapping of command-line options to command-line arguments
    """
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("-p", "--port")
    arguments = {key.replace("_", "-"): value for key, value in vars(argument_parser.parse_args()).items()}
    return arguments

def read_data_from_db(num_records, num_mcu=1, num_i2c_ports=1):
    """
    Read serial data from the database.

    The data is returned as a dictionary with the following structure:

    {
        <mcu_0_id>: {
            <i2c_port_0>: [
                {
                    "date_time": <date_time>,
                    "compensated_temperature": <compensated_temperature>,
                    "compensated_pressure": <compensated_pressure>
                },
                ...
            ],
            <i2c_port_1>: [
                {
                    "date_time": <date_time>,
                    "compensated_temperature": <compensated_temperature>,
                    "compensated_pressure": <compensated_pressure>
                },
                ...
            ]
        },
        <mcu_1_id>: {
            <i2c_port_0>: [
                {
                    "date_time": <date_time>,
                    "compensated_temperature": <compensated_temperature>,
                    "compensated_pressure": <compensated_pressure>
                },
                ...
            ],
            <i2c_port_1>: [
                {
                    "date_time": <date_time>,
                    "compensated_temperature": <compensated_temperature>,
                    "compensated_pressure": <compensated_pressure>
                },
                ...
            ]
        }
    }

    <mcu_0_id>, the ID of the first microcontroller, should be 0. <mcu_1_id>, the ID of the second microcontroller (if any), should be 1. Similarly, <i2c_port_0> and <i2c_port_1> will be 0 and 1, respectively. If two microcontrollers are used and one uses two I2C ports, then the returned dictionary containing the serial data will contain a list for both I2C ports for each microcontroller, even if the other microcontroller uses only one I2C port (so the list will be empty).

    Note that only the most recent 40 records for each microcontroller ID, I2C port pair are returned.

    Arguments:
        num_records: the number of records to read from the database, beginning with the most recent
        num_mcu: the number of microcontrollers (default 1)
        num_i2c_ports: the maximum number of I2C ports used by any microcontroller (default is 1)

    Return:
        a dictionary containing the serial data
    """
    data = {}
    with contextlib.closing(sqlite3.connect(DB_FILE_PATH)) as db_connection:
        with contextlib.closing(db_connection.cursor()) as db_cursor:
            for mcu_id in range(num_mcu):
                for i2c_port in range(num_i2c_ports):
                    result = db_cursor.execute(
                        """
                        SELECT date_time, compensated_temperature, compensated_pressure
                        FROM serial_data
                        WHERE mcu_id = ? AND i2c_port = ?
                        ORDER BY date_time DESC
                        LIMIT ?;
                        """,
                        (mcu_id, i2c_port, num_records)
                    )
                    d = result.fetchall()
                    d.reverse()
                    d = list(map(
                        lambda x: {
                            "date_time": x[0],
                            "compensated_temperature": x[1],
                            "compensated_pressure": x[2],
                        },
                        d
                    ))
                    if mcu_id in data:
                        data[mcu_id][i2c_port] = d
                    else:
                        data[mcu_id] = {
                            i2c_port: d
                        }
    return data

@app.route("/")
def index():
    return flask.make_response(flask.render_template("index.html"))

@app.route("/api/serial_data/")
def serial_data_api():
    num_records = flask.request.args.get("num_records") if "num_records" in flask.request.args else DEFAULT_NUM_RECORDS
    data = read_data_from_db(num_records, 2, 2)
    return data

if __name__ == "__main__":
    # read arguments from command line
    arguments = read_arguments()
    port = DEFAULT_PORT if arguments["port"] is None else int(arguments["port"])

    webbrowser.open(f"http://localhost:{port}")
    app.run("localhost", 8000)

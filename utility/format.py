import argparse
import os
import re
import shutil
import sys

if __name__ == "__main__":
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("-d", "--dir")
    argument_parser.add_argument("-c", "--component-only", action="store_true")
    argument_parser.add_argument("-m", "--main")
    arguments = {key.replace("_", "-"): value for key, value in vars(argument_parser.parse_args()).items()}
    if arguments["component-only"] and arguments["main"]:
        raise ValueError("Cannot specify --component-only with --main.")
    if arguments["dir"]:
        os.chdir(arguments["dir"])
    if "utility" == os.path.split(os.getcwd())[1]:
        os.chdir("..")
    os.makedirs("temperature_pressure_sensor", exist_ok=True)
    os.makedirs(os.path.join("temperature_pressure_sensor", "components"), exist_ok=True)
    os.makedirs(os.path.join("temperature_pressure_sensor", "components", "read_temperature_pressure"), exist_ok=True)
    os.makedirs(os.path.join("temperature_pressure_sensor", "components", "read_temperature_pressure", "include"), exist_ok=True)
    shutil.copy(os.path.join("CMakeLists", "root", "CMakeLists.txt"), "temperature_pressure_sensor")
    shutil.copy(os.path.join("CMakeLists", "read_temperature_pressure", "CMakeLists.txt"), os.path.join("temperature_pressure_sensor", "components", "read_temperature_pressure"))
    shutil.copy(os.path.join("read_temperature_pressure", "read_temperature_pressure.c"), os.path.join("temperature_pressure_sensor", "components", "read_temperature_pressure"))
    shutil.copy(os.path.join("read_temperature_pressure", "read_temperature_pressure.h"), os.path.join("temperature_pressure_sensor", "components", "read_temperature_pressure", "include"))
    if not arguments["component-only"]:
        path_to_main = None
        if arguments["main"]:
            path_to_main = arguments["main"]
        else:
            path_to_main = os.path.join("main", "main.c")
        os.makedirs(os.path.join("temperature_pressure_sensor", "main"), exist_ok=True)
        shutil.copy(path_to_main, os.path.join("temperature_pressure_sensor", "main"))
        data = None
        with open(os.path.join("CMakeLists", "main", "CMakeLists.txt")) as file:
            data = file.read()
        pattern = re.compile(r"idf_component_register\(SRCS \"\w+\.c\"")
        data = re.sub(pattern, f"idf_component_register(SRCS \"{os.path.split(path_to_main)[1]}\"", data)
        with open(os.path.join("temperature_pressure_sensor", "main", "CMakeLists.txt"), "w") as file:
            file.write(data)
    sys.exit(0)

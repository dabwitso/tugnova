#!/usr/bin/env python3

import json
import sys
import subprocess, shlex

jsonFile = "/home/nvidia/Autoware/param/respawn_node.json"
jsonf = open(jsonFile, "r")
data = json.load(jsonf)

base_directory = data["base_directory"]
shell_directory = data["shell_directory"]
param_yaml = data["param_yaml"]
custom_yaml = data["custom_yaml"]

runsystem_env = data["runsystem_env"]
envf = open(runsystem_env, "r")

courseIdFile = "/home/nvidia/Autoware/current_lane_id"
coursef = open(courseIdFile, "r")
area_list_file = "/home/nvidia/Autoware/ros/renkei/" + coursef.readline().strip().split("_")[0] + "/AREA_LIST/arealists.txt"



nodeName = ""
command = sys.argv[1]
if command == "help":
    print("respawn_node.py help:            Show Help")
    print("respawn_node.py list:            Show Target Node List")
    print("respawn_node.py info [NodeName]: Show Information")
    print("respawn_node.py [NodeName]:      Restart Target Node")
    sys.exit(0)
elif command == "info":
    nodeName = sys.argv[2]
else:
    nodeName = command



for target in data["target"]:
    if command == "list":
        print(target["node"])
        continue

    if target["node"] == nodeName:
        processes = target["respawn_process"]
        if command == "info":
            print("Name: " + target["node"])
            print("Process: ")
            for process in processes:
                print("   " + process)
            break

        for process in processes:
            process = process.replace("{", "")
            process = process.replace("}", "")
            process = process.replace("$BASE_DIR", base_directory)
            process = process.replace("$SHELL_DIR", shell_directory)
            process = process.replace("$PARAM_YAML", param_yaml)
            process = process.replace("$CUSTOM_YAML", custom_yaml)
            for line in envf:
                key,value = line.strip().split("=")
                process = process.replace("$" + key, value)
            envf.seek(0, 0)
            subprocess.call(process, shell=True)
        break



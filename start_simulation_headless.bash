#!/bin/bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)

# Only the following arguments are accepted: beaumont, waterwitch
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 {beaumont|waterwitch}"
    exit 1
fi

VEHICLE=$1

if [ "$VEHICLE" != "beaumont" ] && [ "$VEHICLE" != "waterwitch" ]; then
    echo "Invalid argument: $VEHICLE"
    echo "Usage: $0 {beaumont|waterwitch}"
    exit 1
fi
if [ "$VEHICLE" == "beaumont" ]; then
    (trap 'kill 0' SIGINT; . initialize_ros_communication.bash beaumont & gz sim -s --headless-rendering -r -v 4 Worlds/BeaumontOceanWorld.sdf)
elif [ "$VEHICLE" == "waterwitch" ]; then
    (trap 'kill 0' SIGINT; . initialize_ros_communication.bash waterwitch & gz sim -s --headless-rendering -r -v 4 Worlds/WaterwitchOceanWorld.sdf)
fi
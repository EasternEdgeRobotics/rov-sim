#!/bin/bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)

(trap 'kill 0' SIGINT; . initialize_ros_communication.bash & gz sim -s --headless-rendering -r -v 4 OceanWorld.sdf)
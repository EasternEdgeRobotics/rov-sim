#!/bin/bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)

while true; do
    read -p "Which ROV do you want to use? (0 for waterwitch, 1 for beaumont): " choice
    if [ "$choice" == "0" ]; then
        VEHICLE="waterwitch"
        break
    elif [ "$choice" == "1" ]; then
        VEHICLE="beaumont"
        break
    else
        echo "Invalid choice. Please enter 0 or 1."
    fi
done

WORLDS=($(ls worlds))
while true; do
    echo "Available worlds:"
    for i in "${!WORLDS[@]}"; do
        echo "$i: ${WORLDS[$i]}"
    done
    read -p "Select a world by index: " world_choice
    if [[ "$world_choice" =~ ^[0-9]+$ ]] && [ "$world_choice" -ge 0 ] && [ "$world_choice" -lt "${#WORLDS[@]}" ]; then
        WORLD="${WORLDS[$world_choice]}"
        break
    else
        echo "Invalid choice. Please enter a valid index."
    fi
done

echo "Starting simulation with $VEHICLE in $WORLD"

(trap 'kill 0' SIGINT; . initialize_ros_communication.bash $VEHICLE & gz sim -s --headless-rendering -r -v 4 worlds/$WORLD) &

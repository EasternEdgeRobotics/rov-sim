# rov-sim

This repository demonstrates an implementation of Gazebo Harmonic for simulating underwater remotely operated vehicles (ROVs). It's purpose is to facilitate simulation of underwater ROVs as quickly as possible using open source software. This may help to improve safety, facilitate design validation, increase testing time.

## Table of Contents
- [Requirements](#required-software)
- [Running the Simulation](#running-the-simulation)
    - [Running the Simulation as a Headless Background Process](#headless-background-process)
- [Features](#features)
- [License](#license)

## Prerequisite Software
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [ROS-Gazebo Bridging Libraries](https://gazebosim.org/docs/harmonic/ros_installation/) 

Note: ROS2 Jazzy and the ROS-Gazebo bridging libraries are not required other than to facilitate the use of an existing ROS2 framework for controlling the real ROV.    

## Running the Simulation
To run the simulation, follow these steps:

1. Clone this repository

2. Install the prerequisite software as mentioned in the [Required Software](#required-software) section.

3. Navigate to this repository clone and set the Gazebo Plugin Path
        ```sh
        export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)
        ```
4. Launch the simulation:
        ```sh
        gz sim -v 4 OceanWorld.sdf
        ```

### Headless Mode
If you want to run the simulation in headless mode on a remote machine, follow these steps:

1. SSH into your remote machine

2. Follow steps 1 and 2 of the [Running the Simulation](#running-the-simulation) section.

3. Enter super-user mode. Running the sim in super-user mode overcomes issues pertaining to recieving data on ros-topics. 
        ```sh
        su
        ```

2. Navigate to this repository clone and run the following script (or make it a systemctl process if you like).
        ```sh
        . start_simulation_headless.bash
        ```


The sim should now be running as a service. To pause/unpause the simulation, run:
    ```sh
    gz service -s /world/<WORLD NAME>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: <true or false>'
    ```

To stop the background service, run:
    ```sh
    sudo systemctl stop rov_sim
    ```

## Features
This repository includes the following features:
- Modified buoyancy plugin making it possible to specify the volume and center of volume for models. 
- Modified thruster plugin based on the [Blue Robotics T200 Thrusters](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/). The plugin makes it possible to choose between different thruster voltages.
- Custom claw plugin to specify left and right claw locations and allowed rotation.
- An overall implementation of a simulated ROV.

## License
This project is licensed under the GPL-3.0 License. See the [LICENSE](./LICENSE) file for more details.

For a list of open source components included, see [3rd-party-licenses.txt](./third-party-licenses.txt).
# rov-sim

This repository demonstrates an implementation of Gazebo Harmonic for simulating underwater remotely operated vehicles (ROVs). It's purpose is to facilitate simulation of underwater ROVs as quickly as possible using open source software. This may help to improve safety, facilitate design validation, and increase testing time.

## Table of Contents
- [Prerequisite Software](#prerequisite-software)
- [Running the Simulation](#running-the-simulation)
    - [Running the Simulation in Headless Mode](#headless-mode)
- [Features](#features)
- [License](#license)

## Prerequisite Software
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [ROS-Gazebo Bridging Libraries](https://gazebosim.org/docs/harmonic/ros_installation/) 

Note: ROS2 Jazzy and the ROS-Gazebo bridging libraries are not required other than to facilitate the use of an existing ROS2 framework for controlling simulated ROVs.    

## Running the Simulation
To run the simulation, follow these steps:

1. Clone this repository

2. Install the prerequisite software as mentioned in the [Prerequistie Software](#prerequisite-software) section.

3. Navigate to this repository clone and set the Gazebo Plugin Path:
        ```
        export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)
        ```
4. Launch the simulation:
        ```
        gz sim -v 4 worlds/<world file (ex. BeaumontOceanWorld.sdf)>
        ```
   
The sim should now be running. To pause/unpause the simulation in the command line, run:
```
gz service -s /world/<WORLD NAME>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: <true or false>'
```

You may run `ros2 topic list` to see the list of ROS topics on which the robot communicates (also see [the list of ros topics](./initialize_ros_communication.bash)). 

### Headless Mode
To run this simulation headless mode on a remote machine, follow these steps:

1. SSH into your remote machine

2. Follow steps 1 and 2 of the [Running the Simulation](#running-the-simulation) section.

3. Navigate to this repository clone and run the following script:
        ```
        bash start_simulation_headless.bash 
        ```

## Features
This repository includes the following features:
- Modified buoyancy plugin making it possible to specify the volume and center of volume for models. 
- Modified thruster plugin based on the [Blue Robotics T200 Thrusters](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/). The plugin makes it possible to choose between different thruster voltages.
- Custom claw plugin to specify left and right claw locations and allowed range of motion.
- Custom servo motor plugin 
- ROV models and associated world files.

## License
This project is licensed under the Apache 2.0 License. See the [LICENSE](./LICENSE) file for more details.

For a list of open source components included, see [3rd-party-licenses.txt](./third-party-licenses.txt).

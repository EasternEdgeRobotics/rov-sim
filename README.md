# Simulation
Welcome to the Eastern Edge Robotics ROV Simulation. 
## How to Run
  
1. [Ensure Gazebo Harmonic is installed](https://gazebosim.org/docs/harmonic/install_ubuntu/)
2. Ensure that Gazebo Can find the custom plugins created in this world by typing 
    `export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)`
3. Run `gz sim -v 4 OceanWorld.sdf`
    
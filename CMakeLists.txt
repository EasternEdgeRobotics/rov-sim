# CMake file adapted from https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/plugin/hello_world

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# Find the Gazebo CMake package, which allows for using custom CMake functions provided by Gazebo
# See https://github.com/gazebosim/gz-cmake
find_package(gz-cmake3 REQUIRED)

# Set the project name
project(EER_SIMULAITON_PLUGINS)

# Find the gz-plugin2, ensuring that it contains the plugin registering functionality
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

# Find the gz-sim8 package, which includes the Gazebo C++ libraries and headers  
gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# Add the plugin libraries
add_library(EER_Buoyancy SHARED src/EER_Buoyancy.cc include/EER_Buoyancy.hh)
add_library(EER_Thruster SHARED src/EER_Thruster.cc include/EER_Thruster.hh)
add_library(EER_Claws SHARED src/EER_Claws.cc include/EER_Claws.hh)
add_library(EER_Hydrodynamics SHARED src/EER_Hydrodynamics.cc include/EER_Hydrodynamics.hh)
add_library(EER_Servo SHARED src/EER_Servo.cc include/EER_Servo.hh)

# Add include directories for the plugins
target_include_directories(EER_Buoyancy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(EER_Thruster PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(EER_Claws PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(EER_Hydrodynamics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(EER_Servo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set the C++ standard to C++17 and link the plugin libraries to Gazebo libraries
foreach(plugin EER_Buoyancy EER_Thruster EER_Claws EER_Hydrodynamics EER_Servo)
  set_property(TARGET ${plugin} PROPERTY CXX_STANDARD 17)
  target_link_libraries(${plugin}
    PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
  set_target_properties(${plugin} PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/plugins)
endforeach()

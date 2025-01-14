cmake_minimum_required(VERSION 3.5)
project(hector_control)

# Define the HARDWARE option
option(HARDWARE "Build with hardware interface" OFF)

include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_CXX_FLAGS "-fPIC")
set(CMAKE_CXX_STANDARD 14)

##############
# Controller #
##############

add_subdirectory(./hector_system/third_party/qpOASES)
find_package(Eigen3 REQUIRED)

include_directories("./hector_system/include")
include_directories("/usr/local/include/eigen3")
set(CONTROLLER_DEPENDENCIES qpOASES Eigen3::Eigen)

file(GLOB_RECURSE control_files
    "./hector_system/include/*/*.h"
    "./hector_system/src/*/*.cpp"
    "./hector_system/ConvexMPC/*.cpp"
    "./hector_system/include/*/*/*.h" 
    "./hector_system/PDController/*.cpp"
)

add_library(controller_lib ${control_files})
target_link_libraries(controller_lib ${CONTROLLER_DEPENDENCIES})

########################
# Simulation interface #
########################
add_subdirectory(Interface/python_interface/pybind11) # build pybind11
include_directories(Interface/python_interface)
include_directories(Interface/python_interface/pybind11/include/pybind11)
pybind11_add_module(${PROJECT_NAME} Interface/python_interface/hector_controller_pybind.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE controller_lib)

######################
# Hardware interface #
######################
if(HARDWARE)
    message(STATUS "Building with hardware interface")

    find_package(realsense2 REQUIRED)
    include_directories(${realsense_INCLUDE_DIR})

    find_package(SDL2 REQUIRED)
    include_directories(${SDL2_INCLUDE_DIRS})

    include_directories("./Interface/HW_interface/include/sdk/include")
    link_directories("./Interface/HW_interface/include/sdk/lib")

    set(DEPENDENCIES realsense2 SDL2)
    set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

    file(GLOB_RECURSE hw_files
        "./Interface/HW_interface/include/*/*.h"
        "./Interface/HW_interface/src/*/*.cpp"
        "./Interface/HW_interface/src/stateestimator/PoseData.cpp"
    )

    add_library(hw_interface_lib ${hw_files})
    target_link_libraries(hw_interface_lib ${EXTRA_LIBS} ${DEPENDENCIES})

    # Hardware executables
    add_executable(biped_ctrl scripts/run_hardware.cpp)
    target_link_libraries(biped_ctrl controller_lib hw_interface_lib)
else()
    message(STATUS "Building without hardware interface")
endif()
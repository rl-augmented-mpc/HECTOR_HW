cmake_minimum_required(VERSION 3.5)
project(quadruped_control)

include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS "-O3 -ggdb -std=gnu99 -I -fPIC")
set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_BINARY_DIR})

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

######################
# Hardware interface #
######################

find_package(realsense2 REQUIRED)
include_directories(${realsense_INCLUDE_DIR})

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

include_directories("./Interface/HW_interface/include/sdk/include")
link_directories("./Interface/HW_interface/include/sdk/lib")

set(DEPENDENCIES realsense2 SDL2)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

set(CMAKE_CXX_FLAGS "-O3")


file(GLOB_RECURSE hw_files
"./Interface/HW_interface/include/*/*.h"
"./Interface/HW_interface/src/*/*.cpp"
"./Interface/HW_interface/src/stateestimator/PoseData.cpp"
)

add_library(hw_interface_lib ${hw_files})
target_link_libraries(hw_interface_lib ${EXTRA_LIBS} ${DEPENDENCIES})


###############
# Executables #
###############

add_executable(biped_ctrl run/run_hardware.cpp)
target_link_libraries(biped_ctrl controller_lib hw_interface_lib)
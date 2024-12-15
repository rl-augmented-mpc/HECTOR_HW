
cmake_minimum_required(VERSION 3.5)
project(hector_interface_hw)



set(CMAKE_C_FLAGS "-O3 -ggdb -std=gnu99 -I -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3")


cmake_minimum_required(VERSION 3.5)



###########################################

include_directories(${CMAKE_BINARY_DIR})


include_directories("../../hector_system/install/include")
link_directories("../../hector_system/build/install/lib")

#########################################################

find_package(realsense2 REQUIRED)
include_directories(${realsense_INCLUDE_DIR})

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

set(DEPENDENCIES realsense2 SDL2)

# set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

#########################################################

add_executable(biped_ctrl src/main.cpp)
link_libraries(biped_ctrl PRIVATE ${EXTRA_LIBS} ${DEPENDENCIES} hector_system)



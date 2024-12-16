cmake_minimum_required(VERSION 3.5)
project(quadruped_control)

include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS "-O3 -ggdb -std=gnu99 -I -fPIC")
set(CMAKE_CXX_STANDARD 14)

# set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall")

include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(third_party/qpOASES)
#add_subdirectory(third_party/JCQP)
include_directories("../../hector_system/include")
include_directories("/usr/local/include/eigen3") 
include_directories("../../hector_system/include/sdk/include")
link_directories("../../hector_system/include/sdk/lib")

find_package(realsense2 REQUIRED)
include_directories(${realsense_INCLUDE_DIR})

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

set(DEPENDENCIES realsense2 SDL2)


set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

set(CMAKE_CXX_FLAGS "-O3")


file(GLOB_RECURSE sources
"../../hector_system/include/*/*.h"
"../../hector_system/src/*/*.cpp"
"../../hector_system/ConvexMPC/*.cpp"
"../../hector_system/include/*/*/*.h" 
"../../hector_system/PDController/*.cpp"

"include/*/*.h"
"src/*/*.cpp"
)

add_library(src ${sources} "src/stateestimator/PoseData.cpp")
target_link_libraries(src ${EXTRA_LIBS} ${DEPENDENCIES})

add_executable(biped_ctrl src/main.cpp)
target_link_libraries(biped_ctrl ${EXTRA_LIBS} ${DEPENDENCIES} src qpOASES)

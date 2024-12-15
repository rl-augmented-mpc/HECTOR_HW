# if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#     set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Default install path" FORCE)
# endif()

cmake_minimum_required(VERSION 3.5)
project(hector_system)



set(CMAKE_C_FLAGS "-O3 -ggdb -std=gnu99 -I -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)
# set(EXTRA_LIBS libunitree_legged_sdk_${ARCH}.so)
set(CMAKE_CXX_FLAGS "-O3")


include_directories(${CMAKE_BINARY_DIR})
include_directories(../../hector_system/third_party/qpOASES/include)
include_directories("/usr/local/include/eigen3") 
include_directories(include/sdk/include)


link_directories(include/sdk/lib)



file(GLOB_RECURSE sources
"include/*/*.h"
"include/*/*/*.h"
"src/*/*.cpp"
"ConvexMPC/*.cpp"
"PDController/*.cpp"
)

add_library(${PROJECT_NAME} ${sources} src/common/PoseData.cpp)
link_libraries(${PROJECT_NAME} ${EXTRA_LIBS})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        EXPORT MyPackageTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
cmake_minimum_required(VERSION 3.0.2)
project(cpp_timer)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Specify additional locations of header files
include_directories(
  include
)

## Declare a C++ library
add_library(cpp_timer SHARED src/Timer.cpp)

add_executable(example src/example.cpp)
target_link_libraries(example cpp_timer)

#############
## Install ##
#############

## Mark libraries for installation
install(TARGETS cpp_timer 
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

## Mark header files for installation
install(FILES include/${PROJECT_NAME}/Timer.h 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
)

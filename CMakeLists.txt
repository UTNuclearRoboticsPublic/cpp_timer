cmake_minimum_required(VERSION 3.0.2)
project(cpp_timer)

## Compile as C++14
add_compile_options(-std=c++14)

## Boost required for function name macros
find_package(Boost REQUIRED COMPONENTS system)

## Specify location of header files
include_directories(
  include
)

## Declare the C++ library
add_library(cpp_timer SHARED src/Timer.cpp)
target_link_libraries(cpp_timer -lpthread)

## Executable for demonstration purposes
add_executable(example src/example.cpp)
target_link_libraries(example cpp_timer)
target_compile_options(example PRIVATE -O0 -g) # remove optimizations and add debug symbols

#############
## Install ##
#############

## Mark libraries for installation
install(
  TARGETS cpp_timer  
  EXPORT cpp_timerConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

## Install the config file associated with the cpp_timer lib
install(
  EXPORT cpp_timerConfig
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME}
  NAMESPACE cpp_timer::
)

## Mark header files for installation
install(
  FILES include/${PROJECT_NAME}/Timer.h 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
)


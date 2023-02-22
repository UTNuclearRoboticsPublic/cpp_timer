cmake_minimum_required(VERSION 3.0.2)
project(cpp_timer)
set(version 1.0)

## Compile as C++17
add_compile_options(-std=c++17)

## Boost required for function name macros
find_package(Boost REQUIRED COMPONENTS system)

## Specify location of header files
include_directories(
  include
)

## Declare the C++ library
add_library(${PROJECT_NAME} SHARED src/Timer.cpp src/Ticker.cpp)
target_link_libraries(${PROJECT_NAME} -lpthread)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -Wextra -Wpedantic -Wconversion)

## Executable for demonstration purposes
add_executable(example src/example.cpp)
target_link_libraries(example ${PROJECT_NAME})
target_compile_options(example PRIVATE -g) # add debug symbols

#############
## Install ##
#############

## Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING
  PATTERN "*.h" 
)

## Mark libraries for installation
install(TARGETS  
  ${PROJECT_NAME}
  EXPORT   "${PROJECT_NAME}Targets"
  LIBRARY  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
  INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

## Install the targets file associated with the lib
install(EXPORT 
  "${PROJECT_NAME}Targets"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/cmake/
)

## Configure the config file to have the necessary defines
configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  @ONLY
)

## Finally install the config file so that cmake can find it
install(FILES
  "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/cmake
)

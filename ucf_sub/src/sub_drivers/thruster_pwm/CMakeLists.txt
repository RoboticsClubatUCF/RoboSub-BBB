cmake_minimum_required(VERSION 2.8.3)
project(thruster_pwm)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/../cmake_modules" {CMAKE_MODULE_PATH} )
set(BlackLib /home/ness/BlackLib/v3_0)
find_package(catkin REQUIRED)

## Declare external dependencies used by packages making use of this package.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thruster_pwm
)

include_directories(include ${catkin_INCLUDE_DIRS} ${BlackLib})

add_library(thruster_pwm src/thruster_pwm.cpp)
target_link_libraries(thruster_pwm BlackLib)

## Install libraries and runtime dependencies
install(TARGETS thruster_pwm 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


cmake_minimum_required(VERSION 2.8.3)
project(ampoule_launcher)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -g -Og")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrs_msgs
  mrs_lib

)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp mrs_lib std_msgs mrs_msgs 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(ampoule_launcher src/launcher.cpp)

add_dependencies(ampoule_launcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ampoule_launcher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(worldlib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  tf2
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf2
  DEPENDS mysqlclient
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Source files for the library
set(WORLDLIB_SOURCE
  src/geometry/Orientation.cpp
  src/geometry/Pose.cpp
  src/geometry/Position.cpp
  src/remote/Client.cpp
  src/remote/SpatialWorldClient.cpp
  src/remote/SqlClient.cpp
  src/world/Item.cpp
  src/world/Object.cpp
  src/world/PlacementSurface.cpp
  src/world/PointOfInterest.cpp
  src/world/Room.cpp
  src/world/Surface.cpp
  src/world/World.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${WORLDLIB_SOURCE})

## Add message build dependencies (needed for source build)
#add_dependencies(${PROJECT_NAME}
#  rail_pick_and_place_msgs_gencpp
#)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mysqlclient
)

# REMOVE THIS
add_executable(test_node
  src/test.cpp
)
target_link_libraries(test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

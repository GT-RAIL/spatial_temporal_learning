cmake_minimum_required(VERSION 2.8.3)
project(world_item_observer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  carl_dynamixel
  graspdb
  move_base_msgs
  rail_manipulation_msgs
  roscpp
  std_srvs
  tf2
  tf2_ros
  worldlib
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  nodes/${PROJECT_NAME}.cpp
  src/ItemObserver.cpp
)
add_executable(carl_auto_observer
  nodes/carl_auto_observer.cpp
  src/CarlAutoObserver.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(carl_auto_observer
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} carl_auto_observer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

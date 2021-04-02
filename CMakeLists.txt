cmake_minimum_required(VERSION 3.1)
project(quanergy_client_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(QuanergyClient REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# finding FLANN doesn't work on 18.04 but is required on 20.04
# we do the find but without required
find_package(FLANN)

# Workaround required due to a bug in VTK6 for Ubuntu 16.04
# See https://bugs.launchpad.net/ubuntu/+source/vtk6/+bug/1573234
if (NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

## Declare a cpp executable
add_executable(client_node 
  src/client_node.cpp
  ${project_HEADERS})

## Specify libraries to link a library or executable target against
 target_link_libraries(client_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   quanergy_client
 )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY settings/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/settings
  )


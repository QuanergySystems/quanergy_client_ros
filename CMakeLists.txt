cmake_minimum_required(VERSION 3.5)
project(quanergy_client_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#########################
## System dependencies ##
#########################

## find ament_cmake for ROS2
find_package(ament_cmake QUIET)

if(NOT ament_cmake_FOUND)
  ## Find catkin macros and libraries for ROS1
  find_package(catkin QUIET COMPONENTS
    roscpp
    sensor_msgs
    pcl_ros
  )
endif()

## System dependencies are found with CMake's conventions
find_package(QuanergyClient REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Boost REQUIRED COMPONENTS program_options)

# finding FLANN doesn't work on 18.04 but is required on 20.04
# we do the find but without required
find_package(FLANN QUIET)

if(ament_cmake_FOUND)
  message("ament_cmake found; executing ROS2 build")
  add_definitions(-DROS2_FOUND)

  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
elseif(catkin_FOUND)
  message("catkin found; executing ROS1 build")

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
  )
else()
  message(FATAL_ERROR "no build system found")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${PCL_INCLUDE_DIRS}
)

if(ament_cmake_FOUND)
  include_directories(
    ${rclcpp_INCLUDE_DIRS}
  )
elseif(catkin_FOUND)
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
endif()


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

## Declare a cpp executable
add_executable(client_node 
  src/client_node.cpp
  ${project_HEADERS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(client_node 
  ${PCL_LIBRARIES}
  quanergy_client
)

if(ament_cmake_FOUND)
  ament_target_dependencies(client_node rclcpp sensor_msgs pcl_conversions)
elseif(catkin_FOUND)
  target_link_libraries(client_node
    ${catkin_LIBRARIES}
  )
endif()

#############
## Install ##
#############

if(ament_cmake_FOUND)
  ## Mark executables and/or libraries for installation
  install(TARGETS client_node DESTINATION lib/${PROJECT_NAME})

  # Mark other files for installation (e.g. launch and bag files, etc.)
  install(FILES launch/client.launch.py
    DESTINATION share/quanergy_client_ros
  )

  install(DIRECTORY settings/
    DESTINATION settings
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()

elseif(catkin_FOUND)
  ## Mark executables and/or libraries for installation
  install(TARGETS client_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  # Mark other files for installation (e.g. launch and bag files, etc.)
  install(FILES launch/client.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

  install(DIRECTORY settings/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/settings
  )
endif()

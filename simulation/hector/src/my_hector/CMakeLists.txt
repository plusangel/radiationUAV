cmake_minimum_required(VERSION 2.8.3)
project(my_hector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  actionlib_msgs
  actionlib
  genmsg
  #message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  HoverHeight.srv
#)

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY
   action
   FILES
   changeHeading.action
   liftDistance.action
   hoverDistance.action
   hoverHeight.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_hector
   CATKIN_DEPENDS actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(my_hector
#   src/${PROJECT_NAME}/my_hector.cpp
# )

## Declare a cpp executable
add_executable(hover src/hover.cpp)
add_executable(turn src/turn.cpp)
#add_executable(obstacleAvoidance1 src/obstacleAvoidance_v1.cpp)
add_executable(obstacleAvoidance2 src/obstacleAvoidance_v2.cpp)
add_executable(changeHeadingServer src/changeHeadingServer.cpp)
add_executable(changeHeadingClient src/changeHeadingClient.cpp)
add_executable(liftDistanceServer src/liftDistanceServer.cpp)
add_executable(liftDistanceClient src/liftDistanceClient.cpp)
add_executable(hoverDistanceServer src/hoverDistanceServer.cpp)
add_executable(hoverDistanceClient src/hoverDistanceClient.cpp)
add_executable(hoverHeightServer src/hoverHeightServer.cpp)
add_executable(hoverHeightClient src/hoverHeightClient.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(hover my_hector_generate_messages_cpp)
#add_dependencies(obstacleAvoidance1 my_hector_generate_messages_cpp)
add_dependencies(obstacleAvoidance2 my_hector_generate_messages_cpp)
add_dependencies(turn my_hector_generate_messages_cpp)
add_dependencies(changeHeadingServer ${my_hector_EXPORTED_TARGETS})
add_dependencies(changeHeadingClient ${my_hector_EXPORTED_TARGETS})
add_dependencies(liftDistanceServer ${my_hector_EXPORTED_TARGETS})
add_dependencies(liftDistanceClient ${my_hector_EXPORTED_TARGETS})
add_dependencies(hoverDistanceServer ${my_hector_EXPORTED_TARGETS})
add_dependencies(hoverDistanceClient ${my_hector_EXPORTED_TARGETS})
add_dependencies(hoverHeightServer ${my_hector_EXPORTED_TARGETS})
add_dependencies(hoverHeightClient ${my_hector_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(hover
   ${catkin_LIBRARIES}
)

#target_link_libraries(obstacleAvoidance1
#   ${catkin_LIBRARIES}
#)

target_link_libraries(obstacleAvoidance2
   ${catkin_LIBRARIES}
)

target_link_libraries(turn
   ${catkin_LIBRARIES}
)

target_link_libraries(changeHeadingServer
   ${catkin_LIBRARIES}
)

target_link_libraries(changeHeadingClient
   ${catkin_LIBRARIES}
)

target_link_libraries(liftDistanceServer
   ${catkin_LIBRARIES}
)

target_link_libraries(liftDistanceClient
   ${catkin_LIBRARIES}
)

target_link_libraries(hoverDistanceServer
   ${catkin_LIBRARIES}
)

target_link_libraries(hoverDistanceClient
   ${catkin_LIBRARIES}
)

target_link_libraries(hoverHeightServer
   ${catkin_LIBRARIES}
)

target_link_libraries(hoverHeightClient
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS my_hector my_hector_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_hector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

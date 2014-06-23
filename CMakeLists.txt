cmake_minimum_required(VERSION 2.8.3)
project(quadcopter_ctrl)

# WHEN BUILT IN RELEASE MODE THERE IS NO BOUNDS CHECK IN VECTORS !!!
# SO TO DEBUNG AND FIND ERRORS COMMENT THE FOLLOWING "set" LINE  !!!
#set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  genmsg
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
 add_message_files(
   FILES
   NCmsg.msg
   LRTAmsg.msg
   OSmsg.msg
   PGmsg.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   AddTwoInts.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
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
#  LIBRARIES beginner_tutorials
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/quadcopter_ctrl
  include/quadcopter_ctrl/astar
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(beginner_tutorials
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )


add_library(PathPlanningAlg src/PathPlanningAlg.cpp)
add_library(VrpGreedyFW src/VrpGreedyFW.cpp)
add_library(NodeCounting src/NodeCounting.cpp)
add_library(VrpGreedyAstar src/VrpGreedyAstar.cpp)
add_library(LRTAstar src/LRTAstar.cpp)
add_library(CoverAnalysis src/CoverAnalysis.cpp)
add_library(FloydWarshall src/FloydWarshall.cpp)
add_library(Utils src/Utils.cpp)
add_library(PatrolGRAPH src/PatrolGRAPH.cpp)

## Declare a cpp executable
# add_executable(beginner_tutorials_node src/beginner_tutorials_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_tutorials_node beginner_tutorials_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(beginner_tutorials_node
#   ${catkin_LIBRARIES}
# )


add_executable(quadcopterRosCtrl src/quadcopterRosCtrl.cpp)
target_link_libraries(quadcopterRosCtrl ${catkin_LIBRARIES})
add_dependencies(quadcopterRosCtrl ${catkin_EXPORTED_TARGETS})
#add_dependencies(quadcopterRosCtrl quadcopter_ctrl_generate_messages_cpp)

add_executable(kernelNode src/kernelNode.cpp)
target_link_libraries(kernelNode ${catkin_LIBRARIES})
add_dependencies(kernelNode ${catkin_EXPORTED_TARGETS})

add_executable(testVRPs src/testVRPs.cpp)
target_link_libraries(testVRPs ${catkin_LIBRARIES})
add_dependencies(testVRPs ${catkin_EXPORTED_TARGETS})

add_executable(quadNodeCount src/quadNodeCount.cpp)
target_link_libraries(quadNodeCount ${catkin_LIBRARIES})
add_dependencies(quadNodeCount ${catkin_EXPORTED_TARGETS})

add_executable(quadLRTA src/quadLRTA.cpp)
target_link_libraries(quadLRTA ${catkin_LIBRARIES})
add_dependencies(quadLRTA ${catkin_EXPORTED_TARGETS})

add_executable(OnlineSearchListener src/OnlineSearchListener.cpp)
target_link_libraries(OnlineSearchListener ${catkin_LIBRARIES})
add_dependencies(OnlineSearchListener ${catkin_EXPORTED_TARGETS})

add_executable(quadPatrolGRAPH src/quadPatrolGRAPH.cpp)
target_link_libraries(quadPatrolGRAPH ${catkin_LIBRARIES})
add_dependencies(quadPatrolGRAPH ${catkin_EXPORTED_TARGETS})



target_link_libraries(quadcopterRosCtrl VrpGreedyFW)
target_link_libraries(quadcopterRosCtrl PathPlanningAlg)

target_link_libraries(kernelNode VrpGreedyFW)
target_link_libraries(kernelNode FloydWarshall)
target_link_libraries(kernelNode Utils)
target_link_libraries(kernelNode VrpGreedyAstar)
target_link_libraries(kernelNode CoverAnalysis)
target_link_libraries(kernelNode -lrt)

target_link_libraries(testVRPs VrpGreedyFW)
target_link_libraries(testVRPs FloydWarshall)
target_link_libraries(testVRPs Utils)
target_link_libraries(testVRPs VrpGreedyAstar)
target_link_libraries(testVRPs CoverAnalysis)
target_link_libraries(testVRPs -lrt)


target_link_libraries(FloydWarshall Utils)

target_link_libraries(quadNodeCount NodeCounting)
target_link_libraries(quadNodeCount PathPlanningAlg)

target_link_libraries(quadLRTA LRTAstar)
target_link_libraries(quadLRTA PathPlanningAlg)

target_link_libraries(OnlineSearchListener CoverAnalysis)

target_link_libraries(quadPatrolGRAPH PatrolGRAPH)
target_link_libraries(quadPatrolGRAPH PathPlanningAlg)
target_link_libraries(quadPatrolGRAPH Utils)


add_dependencies(quadcopterRosCtrl ${catkin_EXPORTED_TARGETS}) #!!!!
add_dependencies(kernelNode ${catkin_EXPORTED_TARGETS})
add_dependencies(testVRPs ${catkin_EXPORTED_TARGETS})
add_dependencies(quadNodeCount ${catkin_EXPORTED_TARGETS})
add_dependencies(quadLRTA ${catkin_EXPORTED_TARGETS})
add_dependencies(OnlineSearchListener ${catkin_EXPORTED_TARGETS})


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
# install(TARGETS beginner_tutorials beginner_tutorials_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

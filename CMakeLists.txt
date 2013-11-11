cmake_minimum_required(VERSION 2.8.3)
project(uw_pr2_gripper_grasp_planner_cluster)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  tf
  visualization_msgs
  moveit_commander
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  manipulation_msgs
  trajectory_msgs
  pr2_controllers_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ClusterBoundingBox.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetPointClusterGraspParams.srv
  FindClusterBoundingBox.srv
  FindClusterBoundingBox2.srv
  GraspPlanning.srv
)

## Generate actionss in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  GraspPlanning.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  moveit_msgs
  manipulation_msgs
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
#  LIBRARIES uw_pr2_gripper_grasp_planner_cluster
#  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(uw_pr2_gripper_grasp_planner_cluster
#   src/${PROJECT_NAME}/uw_pr2_gripper_grasp_planner_cluster.cpp
# )

## Declare a cpp executable
#add_executable(grasp_planning_client_node src/grasp_planning_client.cpp)
add_executable(grasping src/grasping.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(uw_pr2_gripper_grasp_planner_cluster_node uw_pr2_gripper_grasp_planner_cluster_generate_messages_cpp)
add_dependencies(grasping ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(grasp_planning_client_node ${catkin_LIBRARIES})
target_link_libraries(grasping ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#  scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
# install(TARGETS uw_pr2_gripper_grasp_planner_cluster uw_pr2_gripper_grasp_planner_cluster_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uw_pr2_gripper_grasp_planner_cluster.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

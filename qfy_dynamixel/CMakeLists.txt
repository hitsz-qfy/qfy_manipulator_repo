cmake_minimum_required(VERSION 2.8.3)
project(qfy_dynamixel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  roscpp
  rospy
  std_msgs
  message_generation
  std_srvs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find
# )
add_message_files(
	FILES 
	multi_joint_point.msg 
	PbvsErrorStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  BeginGrasp.srv
)

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

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qfy_dynamixel
#  CATKIN_DEPENDS dynamixel_controllers roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(test1_client src/test1_client.cpp)
add_dependencies(test1_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test1_client ${catkin_LIBRARIES})


add_executable(test1_service src/test1_service.cpp)
add_dependencies(test1_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test1_service ${catkin_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  #scripts/trajectory_sub_client.py
  scripts/trajectory_ax_client.py 
  scripts/trajectory_mx_client.py 
  scripts/pub_joint.py
  scripts/kinematic.py
  scripts/track_client.py
  scripts/track.py
  scripts/qfy_joint_trajectory.py
  scripts/mx_position_speed.py
  scripts/ax_position_speed.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS qfy_dynamixel qfy_dynamixel_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_qfy_dynamixel.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

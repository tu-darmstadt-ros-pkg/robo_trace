cmake_minimum_required(VERSION 3.0.2)
project(robo_trace_ui_adaptor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  rosgraph_msgs
  std_srvs
  robo_trace_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    PlaybackMeta.msg
    PlaybackProgression.msg
    StatusIndication.msg
)

add_service_files(
  FILES
    SetPlaybackState.srv
    SetPlaybackTime.srv
    SetPlaybackConfiguration.srv
)

generate_messages(
  DEPENDENCIES 
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    roslib
    std_msgs
    rosgraph_msgs
    std_srvs
    robo_trace_msgs
    message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
    include/${PROJECT_NAME}/ui_adaptor.hpp
    src/ui_adaptor.cpp
)

add_dependencies(
  ${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

add_executable(
  ${PROJECT_NAME}_node
    src/robo_trace_ui_adaptor_node.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node 
    ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS 
    ${PROJECT_NAME}_node
  RUNTIME DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS 
    ${PROJECT_NAME}
  ARCHIVE DESTINATION 
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION 
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION 
    ${CATKIN_GLOBAL_BIN_DESTINATION}
)


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
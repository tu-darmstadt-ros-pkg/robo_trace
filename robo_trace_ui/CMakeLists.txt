cmake_minimum_required(VERSION 3.0.2)
project(robo_trace_ui)

find_package(catkin REQUIRED COMPONENTS
  hector_rviz_overlay
)

find_package(Qt5 REQUIRED Core)
set(CMAKE_AUTOMOC ON)



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS hector_rviz_overlay
#  DEPENDS system_lib
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
    include/${PROJECT_NAME}/robo_trace_ui_display.hpp
    src/robo_trace_ui_display.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*~" EXCLUDE
)

install(
  TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY media 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES plugin_description.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

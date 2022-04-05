cmake_minimum_required(VERSION 3.0.2)
project(robo_trace_evaluation)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories()

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS 
    scripts/dump_arrival_times.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
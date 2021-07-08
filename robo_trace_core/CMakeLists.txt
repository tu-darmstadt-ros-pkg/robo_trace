cmake_minimum_required(VERSION 3.0.2)
project(robo_trace_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wno-unused-variable -Wfatal-errors -fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND 
  CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/"
)

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    nodelet
    pluginlib
    ros_babel_fish
    ddynamic_reconfigure
    robo_trace_plugin_interface
)

find_package(MongoDB REQUIRED)

if (CMAKE_COMPILER_IS_GNUCC)
  add_compile_options(-O1)
endif()

include(CheckIncludeFileCXX)
check_include_file_cxx("mongo/version.h" HAVE_MONGO_VERSION_H)

if(HAVE_MONGO_VERSION_H)
  add_definitions(-DWAREHOUSE_ROS_MONGO_HAVE_MONGO_VERSION_H)
endif()

set(MONGO_EXPORT)

if("${MongoDB_LIBRARIES}" MATCHES "\\.so$")
  set(MONGO_EXPORT MongoDB)
endif()

file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    robo_trace_core
  CATKIN_DEPENDS 
    roscpp
    nodelet
    pluginlib
    ros_babel_fish
    ddynamic_reconfigure
    robo_trace_plugin_interface
  DEPENDS
    ${MONGO_EXPORT}
)


###########
## Build ##
###########

if (NOT MongoDB_EXPOSE_MACROS)
  add_definitions(-DMONGO_EXPOSE_MACROS)
endif()

configure_file(
  "include/${PROJECT_NAME}/config.h.in" 
  "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/config.h"
)

include_directories(
  ${CATKIN_DEVEL_PREFIX}/include
  include
  ${catkin_INCLUDE_DIRS}
  ${MongoDB_INCLUDE_DIR}
)

set(
  SOURCES

    include/${PROJECT_NAME}/persistance/connection.hpp
    src/persistance/connection.cpp

    include/${PROJECT_NAME}/persistance/store.hpp
    src/persistance/store.cpp

    include/${PROJECT_NAME}/persistance/bson_backed_lazy_metadata_container.hpp
    src/persistance/bson_backed_lazy_metadata_container.cpp

    include/${PROJECT_NAME}/processing/performance_measureing_stage_delegate.hpp
    src/processing/performance_measuring_stage_delegate.cpp

    include/${PROJECT_NAME}/processing/marshalling/serializer.hpp
    src/processing/marshalling/serializer.cpp

    include/${PROJECT_NAME}/processing/marshalling/descriptor.hpp
    src/processing/marshalling/descriptor.cpp

    include/${PROJECT_NAME}/processing/invocation/invoker.hpp

    include/${PROJECT_NAME}/processing/invocation/sequential.hpp
    src/processing/invocation/sequential.cpp

    include/${PROJECT_NAME}/processing/forward/pipeline.hpp
    src/processing/forward/pipeline.cpp

    include/${PROJECT_NAME}/processing/forward/descriptor.hpp
    src/processing/forward/descriptor.cpp
    
    include/${PROJECT_NAME}/processing/forward/constructor.hpp
    src/processing/forward/constructor.cpp

    include/${PROJECT_NAME}/utils/smart_ptr_conversions.hpp

    include/${PROJECT_NAME}/orchestrator.hpp
    src/orchestrator.cpp

    src/robo_trace_node.cpp
    src/robo_trace_nodelet.cpp
)

add_library(
  ${PROJECT_NAME} 
    ${SOURCES}
)


add_dependencies(
  ${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
)

add_executable(
  robo_trace_node 
    src/robo_trace_node.cpp
)

target_link_libraries(robo_trace_node ${PROJECT_NAME})


#############
## Install ##
#############

install(
  DIRECTORY 
    cfg
  DESTINATION 
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS 
    ${PROJECT_NAME} 
    robo_trace_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY 
    include/${PROJECT_NAME}/
  DESTINATION 
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING 
    PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(
  FILES 
    nodelet_plugins.xml
  DESTINATION 
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
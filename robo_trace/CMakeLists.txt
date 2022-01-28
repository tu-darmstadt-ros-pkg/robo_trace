cmake_minimum_required(VERSION 3.0.2)
project(robo_trace)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wno-unused-variable -Wfatal-errors -fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#list(APPEND 
#  CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/"
#)

find_package(catkin 
  REQUIRED COMPONENTS
    roscpp
    nodelet
    pluginlib
    ros_babel_fish
    rosgraph_msgs
    ddynamic_reconfigure
    robo_trace_msgs
)

find_package(Boost 
  REQUIRED COMPONENTS
    program_options
    regex
)

#find_package(MongoDB REQUIRED)
find_package(bsoncxx REQUIRED)
find_package(mongocxx REQUIRED)

if (CMAKE_COMPILER_IS_GNUCC)
  add_compile_options(-O1)
endif()

#include(CheckIncludeFileCXX)
#check_include_file_cxx("mongo/version.h" HAVE_MONGO_VERSION_H)

#file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES
    robo_trace
  CATKIN_DEPENDS 
    roscpp
    nodelet
    pluginlib
    ros_babel_fish
    rosgraph_msgs
    ddynamic_reconfigure
    robo_trace_msgs
  DEPENDS
     ${LIBMONGOCXX_EXPORT}
#    ${MONGO_EXPORT}
)


###########
## Build ##
###########

#if (NOT MongoDB_EXPOSE_MACROS)
#  add_definitions(-DMONGO_EXPOSE_MACROS)
#endif()

#configure_file(
#  "include/${PROJECT_NAME}/config.h.in" 
#  "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/config.h"
#)

include_directories(
  ${CATKIN_DEVEL_PREFIX}/include
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
#  ${MongoDB_INCLUDE_DIR}
  ${LIBMONGOCXX_INCLUDE_DIRS}
)

set(
  SOURCES

    include/${PROJECT_NAME}/parameters.hpp

    include/${PROJECT_NAME}/util/options.hpp
    src/util/options.cpp

    include/${PROJECT_NAME}/util/align.hpp
    include/${PROJECT_NAME}/util/smart_ptr_conversions.hpp
    include/${PROJECT_NAME}/util/ts_queue.hpp

    include/${PROJECT_NAME}/storage/options.hpp
    src/storage/options.cpp
    include/${PROJECT_NAME}/storage/container.hpp
    src/storage/container.cpp
    include/${PROJECT_NAME}/storage/connector.hpp
    src/storage/connector.cpp
    include/${PROJECT_NAME}/storage/persistor.hpp
    src/storage/persistor.cpp
    include/${PROJECT_NAME}/storage/persistors/direct.hpp
    src/storage/persistors/direct.cpp
    include/${PROJECT_NAME}/storage/persistors/batch.hpp
    src/storage/persistors/batch.cpp

#[[
    include/${PROJECT_NAME}/storage/query.hpp
    src/storage/query.cpp
    include/${PROJECT_NAME}/storage/result.hpp
    src/storage/result.cpp
]]

    include/${PROJECT_NAME}/processing/mode.hpp
    include/${PROJECT_NAME}/processing/context.hpp
    src/processing/context.cpp
    include/${PROJECT_NAME}/processing/processor.hpp
    src/processing/processor.cpp
    include/${PROJECT_NAME}/processing/descriptor.hpp
    src/processing/descriptor.cpp
    include/${PROJECT_NAME}/processing/plugin.hpp
    src/processing/plugin.cpp
    include/${PROJECT_NAME}/processing/variant.hpp
    src/processing/variant.cpp
    include/${PROJECT_NAME}/processing/constructor.hpp
    src/processing/constructor.cpp

    include/${PROJECT_NAME}/processing/modules/measurement/delegate.hpp
    src/processing/modules/measurement/delegate.cpp

    include/${PROJECT_NAME}/processing/modules/marshalling/forward.hpp
    src/processing/modules/marshalling/forward.cpp
    include/${PROJECT_NAME}/processing/modules/marshalling/backward.hpp
    src/processing/modules/marshalling/backward.cpp
    include/${PROJECT_NAME}/processing/modules/marshalling/descriptor.hpp
    src/processing/modules/marshalling/descriptor.cpp

    include/${PROJECT_NAME}/processing/modules/downsampling/forward_count.hpp
    src/processing/modules/downsampling/forward_count.cpp
    include/${PROJECT_NAME}/processing/modules/downsampling/forward_time.hpp
    src/processing/modules/downsampling/forward_time.cpp
    include/${PROJECT_NAME}/processing/modules/downsampling/descriptor.hpp
    src/processing/modules/downsampling/descriptor.cpp
    include/${PROJECT_NAME}/processing/modules/downsampling/configuration.hpp
    src/processing/modules/downsampling/configuration.cpp

    include/${PROJECT_NAME}/processing/modules/storage/forward.hpp
    src/processing/modules/storage/forward.cpp

 #[[
    include/${PROJECT_NAME}/processing/stage/blob/forward.hpp
    src/processing/stage/blob/forward.cpp
    include/${PROJECT_NAME}/processing/stage/blob/backward.hpp
    src/processing/stage/blob/backward.cpp
    include/${PROJECT_NAME}/processing/stage/blob/descriptor.hpp
    src/processing/stage/blob/descriptor.cpp
]]
    
    include/${PROJECT_NAME}/modes/capture/options.hpp
    src/modes/capture/options.cpp
    include/${PROJECT_NAME}/modes/capture/persistor.hpp
    src/modes/capture/persistor.cpp
     include/${PROJECT_NAME}/modes/capture/recorder.hpp
    src/modes/capture/recorder.cpp
   
    include/${PROJECT_NAME}/modes/replay/options.hpp
    src/modes/replay/options.cpp
    include/${PROJECT_NAME}/modes/replay/loader.hpp
    src/modes/replay/loader.cpp
    include/${PROJECT_NAME}/modes/replay/publisher.hpp
    src/modes/replay/publisher.cpp
    include/${PROJECT_NAME}/modes/replay/time.hpp
    src/modes/replay/time.cpp
    include/${PROJECT_NAME}/modes/replay/player.hpp
    src/modes/replay/player.cpp
    include/${PROJECT_NAME}/modes/replay/cli/player.hpp
    src/modes/replay/cli/player.cpp
     include/${PROJECT_NAME}/modes/replay/service/player.hpp
    src/modes/replay/service/player.cpp

#[[

    include/${PROJECT_NAME}/modes/replay/util.hpp
    src/modes/replay/util.cpp


    #src/robo_trace_node.cpp
    #src/robo_trace_nodelet.cpp
]]
)

add_library(
  robo_trace
    ${SOURCES}
)

add_dependencies(
  robo_trace 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  robo_trace # PRIVATE
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    mongo::mongocxx_shared
    #${MongoDB_LIBRARIES}
    # ${LIBMONGOCXX_LIBRARIES}
)


add_executable(
  record 
    src/modes/capture/record.cpp
)

target_link_libraries(record robo_trace)

add_executable(
  play 
    src/modes/replay/cli/play.cpp
)

target_link_libraries(play robo_trace)

add_executable(
  remote 
    src/modes/replay/service/serve.cpp
)

target_link_libraries(remote robo_trace)


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
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  TARGETS 
    robo_trace
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#[[
install(
  TARGETS 
    record
    play
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)]]

install(
  FILES 
    nodelet_plugins.xml
  DESTINATION 
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
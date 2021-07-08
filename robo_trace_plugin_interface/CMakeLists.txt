cmake_minimum_required(VERSION 3.0.2)
project(robo_trace_plugin_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wno-unused-variable -Wfatal-errors -fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND 
  CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/"
)

find_package(catkin 
  REQUIRED COMPONENTS
    roscpp
    ros_babel_fish
)

find_package(MongoDB REQUIRED)

if (CMAKE_COMPILER_IS_GNUCC)
  add_compile_options(-O1)
endif()

include(CheckIncludeFileCXX)
check_include_file_cxx("mongo/version.h" HAVE_MONGO_VERSION_H)

file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    # Make auto generated config.h file available to other pkgs.
    ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    ros_babel_fish
  DEPENDS
    MongoDB
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
    include/${PROJECT_NAME}/config.h
    include/${PROJECT_NAME}/definitions.hpp

    include/${PROJECT_NAME}/processing/context.hpp
    src/processing/context.cpp

    include/${PROJECT_NAME}/processing/message.hpp
    src/processing/message.cpp

    include/${PROJECT_NAME}/processing/metadata.hpp
    
    include/${PROJECT_NAME}/processing/stage.hpp
    src/processing/stage.cpp

    include/${PROJECT_NAME}/processing/descriptor.hpp
    src/processing/descriptor.cpp

    include/${PROJECT_NAME}/processing/configurable/stage.hpp
    include/${PROJECT_NAME}/processing/configurable/stage.ipp
    include/${PROJECT_NAME}/processing/configurable/descriptor.hpp
    include/${PROJECT_NAME}/processing/configurable/descriptor.ipp

    include/${PROJECT_NAME}/processing/exceptions/unsupported_target_stage_exception.hpp
    src/processing/exceptions/unsupported_target_stage_exception.cpp

    include/${PROJECT_NAME}/plugin.hpp
    src/plugin.cpp

)

add_library(
  ${PROJECT_NAME} SHARED 
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
    ${MongoDB_LIBRARIES}
)


#############
## Install ##
#############

install(
  TARGETS 
    ${PROJECT_NAME}
  ARCHIVE 
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY 
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME 
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY 
    include/${PROJECT_NAME}/
  DESTINATION 
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
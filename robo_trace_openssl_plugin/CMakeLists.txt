cmake_minimum_required(VERSION 3.0.2)
project(robo_trace_openssl_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "-O3 -Wall -Wno-unused-result -Wno-unknown-pragmas -Wno-unused-variable -Wfatal-errors -fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenSSL REQUIRED)

find_package(catkin 
  REQUIRED COMPONENTS
    robo_trace
    roscpp
    pluginlib
    ros_babel_fish
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    robo_trace_openssl_plugin
  CATKIN_DEPENDS 
    robo_trace
    roscpp
    pluginlib
    ros_babel_fish
  DEPENDS
    OPENSSL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

set(
  SOURCES

    include/${PROJECT_NAME}/definitions.hpp
    include/${PROJECT_NAME}/parameters.hpp

    include/${PROJECT_NAME}/key_manager.hpp
    src/key_manager.cpp
    include/${PROJECT_NAME}/robo_trace_openssl_plugin.hpp
    src/robo_trace_openssl_plugin.cpp

    include/${PROJECT_NAME}/chain/configuration.hpp
    src/chain/configuration.cpp
    include/${PROJECT_NAME}/chain/descriptor.hpp
    src/chain/descriptor.cpp
    include/${PROJECT_NAME}/chain/forward.hpp
    src/chain/forward.cpp
    include/${PROJECT_NAME}/chain/validate.hpp
    src/chain/validate.cpp

    include/${PROJECT_NAME}/signature/configuration.hpp
    src/signature/configuration.cpp
    include/${PROJECT_NAME}/signature/descriptor.hpp
    src/signature/descriptor.cpp
    include/${PROJECT_NAME}/signature/forward.hpp
    src/signature/forward.cpp

    include/${PROJECT_NAME}/encryption/full/configuration.hpp
    src/encryption/full/configuration.cpp
    include/${PROJECT_NAME}/encryption/full/descriptor.hpp
    src/encryption/full/descriptor.cpp
    include/${PROJECT_NAME}/encryption/full/forward.hpp
    src/encryption/full/forward.cpp
    include/${PROJECT_NAME}/encryption/full/backward.hpp
    src/encryption/full/backward.cpp

    include/${PROJECT_NAME}/encryption/partial/configuration.hpp
    src/encryption/partial/configuration.cpp
    include/${PROJECT_NAME}/encryption/partial/descriptor.hpp
    src/encryption/partial/descriptor.cpp
    include/${PROJECT_NAME}/encryption/partial/forward.hpp
    src/encryption/partial/forward.cpp
    include/${PROJECT_NAME}/encryption/partial/backward.hpp
    src/encryption/partial/backward.cpp
    
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
    OpenSSL::SSL
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

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  FILES
    plugins.xml
  DESTINATION 
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


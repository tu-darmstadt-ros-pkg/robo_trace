cmake_minimum_required(VERSION 3.0.2)
project(robo_trace_openssl_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wno-unused-variable -Wfatal-errors -fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenSSL 
  REQUIRED
)

find_package(catkin 
  REQUIRED COMPONENTS
  robo_trace_plugin_interface
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
    robo_trace_openssl_plugins
  CATKIN_DEPENDS 
    robo_trace_plugin_interface
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
  HEADERS

    include/${PROJECT_NAME}/definitions.hpp
    include/${PROJECT_NAME}/key_manager.hpp
    include/${PROJECT_NAME}/robo_trace_openssl_plugins.hpp

    include/${PROJECT_NAME}/chain/configuration.hpp
    include/${PROJECT_NAME}/chain/descriptor.hpp
    include/${PROJECT_NAME}/chain/forward.hpp
    include/${PROJECT_NAME}/chain/validate.hpp

    include/${PROJECT_NAME}/signature/configuration.hpp
    include/${PROJECT_NAME}/signature/descriptor.hpp
    include/${PROJECT_NAME}/signature/forward.hpp

    include/${PROJECT_NAME}/encryption/full/configuration.hpp
    include/${PROJECT_NAME}/encryption/full/descriptor.hpp
    include/${PROJECT_NAME}/encryption/full/forward.hpp

    include/${PROJECT_NAME}/encryption/partial/configuration.hpp
    include/${PROJECT_NAME}/encryption/partial/descriptor.hpp
    include/${PROJECT_NAME}/encryption/partial/stage.hpp
    
)

set(
  SOURCES   
  
    src/key_manager.cpp
    src/robo_trace_openssl_plugins.cpp

    src/chain/configuration.cpp
    src/chain/descriptor.cpp
    src/chain/forward.cpp
    src/chain/validate.cpp

    src/signature/configuration.cpp
    src/signature/descriptor.cpp
    src/signature/forward.cpp

    src/encryption/full/configuration.cpp
    src/encryption/full/descriptor.cpp
    src/encryption/full/forward.cpp

    src/encryption/partial/configuration.cpp
    src/encryption/partial/descriptor.cpp
    src/encryption/partial/stage.cpp

)

add_library(
  ${PROJECT_NAME} 
    ${HEADERS} 
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


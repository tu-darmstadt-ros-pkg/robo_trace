cmake_minimum_required(VERSION 3.0.2)
project(robo_trace_msgs)

find_package(catkin 
  REQUIRED
    actionlib_msgs
    message_generation 
    std_msgs 
    geometry_msgs
)

#add_message_files(
#  FILES
#    # TestArray.msg
#)

add_service_files(
  FILES
    SetFloat.srv
    SetConfiguration.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    actionlib_msgs 
    std_msgs 
    geometry_msgs
)
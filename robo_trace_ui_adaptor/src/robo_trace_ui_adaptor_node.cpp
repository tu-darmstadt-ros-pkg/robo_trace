// Ros
#include <ros/ros.h>
// Project
#include "robo_trace_ui_adaptor/ui_adaptor.hpp"


int main(int argc, char** argv) {
   
    //
    ros::init(argc, argv, "robo_trace_ui_adaptor");
    //
    ros::NodeHandle node_handle("~");


    robo_trace_ui_adaptor::RoboTraceUiAdaptor adaptor(node_handle);

    ros::spin();

}
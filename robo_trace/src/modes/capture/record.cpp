// Std
#include <signal.h>
// ROS
#include <ros/ros.h>
// Project
#include "robo_trace/modes/capture/recorder.hpp"  


void onSignal(int signal) {
    (void) signal;
    ros::requestShutdown();
}  

int main(int argc, char** argv) {
    
    // Init ROS.
    ros::init(argc, argv, "trace", ros::init_options::AnonymousName);
    // Route SIGTERM interrupts to a handler for clean shutdown.
    signal(SIGTERM, onSignal);

    ros::NodeHandle m_system_node_handle("robo_trace");
    
    robo_trace::capture::Recorder recorder(m_system_node_handle);
    recorder.initialize(argc, argv);
    
    ros::AsyncSpinner spinner(8);
    spinner.start();

    ros::waitForShutdown();

}

/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Std
#include <signal.h>
#include <functional>
// ROS
#include <ros/ros.h>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/modes/capture/recorder.hpp"  


std::function<void(int)> termination_callback_wrapper;

void onSignal(int signal) {
    termination_callback_wrapper(signal);
}

int main(int argc, char** argv) {
    
    // Init ROS.
    ros::init(argc, argv, "trace", ros::init_options::AnonymousName);
    
    ros::NodeHandle m_system_node_handle("robo_trace");
    
    robo_trace::capture::Recorder recorder(m_system_node_handle);
    
    /**
     * Setup termination signal forwarding for clean termination.
     */

    termination_callback_wrapper = std::bind(
        &robo_trace::capture::Recorder::terminate,
        &recorder,
        std::placeholders::_1
    );

    struct sigaction sig_term_handler = {0};
    sig_term_handler.sa_handler = onSignal;

    sigaction(SIGTERM, &sig_term_handler, NULL);
    sigaction(SIGINT, &sig_term_handler, NULL);

    /**
     * Start the recorder
     */   

    recorder.initialize(argc, argv);

    ros::AsyncSpinner spinner(THREAD_COUNT_ROS);
    spinner.start();

    ros::waitForShutdown();

}

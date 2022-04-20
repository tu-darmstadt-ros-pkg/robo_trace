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
// ROS
#include <ros/ros.h>
// Project
#include "robo_trace/modes/replay/service/player.hpp"

void onSignal(int signal) {
    (void) signal;
    ros::requestShutdown();
}  

int main(int argc, char** argv) {
    
    // Init ROS.
    ros::init(argc, argv, "replay_srv", ros::init_options::AnonymousName);
    // Route SIGTERM interrupts to a handler for clean shutdown.
    signal(SIGTERM, onSignal);

    ros::NodeHandle m_system_node_handle("robo_trace");
    
    robo_trace::replay::RoboTraceReplayService player(m_system_node_handle);
    robo_trace::replay::PlayerBase& base = player;
    base.initialize(argc, argv);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    player.run();

    ros::waitForShutdown();

}
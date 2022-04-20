/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// ROS
#include "ros/ros.h"
// Nodelet
#include "nodelet/loader.h"
// Project
#include "robo_trace_core/parameters.hpp"
#include "robo_trace_core/orchestrator.hpp"

/**
 * Entry point to the framework, when starting as a standart node.
 * 
 * @param argc the number of arguments to the progam
 * @param argv a pointer to the array, containing the arguments.
 *  
 * @returns returns always 0.
 */
int main(int argc, char **argv) {

    ros::init(argc, argv, ROBO_TRACE_NODE_NAME);

/*
    nodelet::V_string nargv;
    nodelet::M_string remap(ros::names::getRemappings());

    std::string nodelet_name = ros::this_node::getName();

    nodelet::Loader nodelet;
    nodelet.load(nodelet_name, "robo_trace/RoboTraceRecorderNodelet", remap, nargv);
*/

    ros::NodeHandle node_handle("robo_trace");

    robo_trace::RoboTraceOrchestrator orchestrator;
    orchestrator.initialize(node_handle, node_handle);

    ros::AsyncSpinner spinner(8);
    spinner.start();

    ros::waitForShutdown();
 //   ros::spin();
    return 0;
}
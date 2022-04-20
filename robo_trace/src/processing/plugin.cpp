/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/plugin.hpp"


namespace robo_trace::processing {

Plugin::Plugin(const std::string& name) 
: m_name(name) {
    //
}

Plugin::~Plugin() = default;


const std::string& Plugin::getName() const {
    return m_name;
}

const std::vector<Descriptor::Ptr>& Plugin::getModules() const {
    return m_modules;
}

ros::NodeHandle& Plugin::getSystemNodeHandle() {
    return m_system_node_handle;
}

ros::NodeHandle& Plugin::getPluginNodeHandle() {
    return m_plugin_node_handle;
}

ros::NodeHandle& Plugin::getStageNodeHandle() {
    return m_stage_node_handle;
}

void Plugin::initialize(ros::NodeHandle& system_node_handle, ros::NodeHandle& plugin_node_handle, ros::NodeHandle& stage_node_handle) {

    /*
        Setup node handles.
    */

    m_system_node_handle = system_node_handle;
    m_plugin_node_handle = ros::NodeHandle(plugin_node_handle, m_name);
    m_stage_node_handle = stage_node_handle;

    /*
        Setup the plugin itself.
    */

    m_modules = setup();

}

}
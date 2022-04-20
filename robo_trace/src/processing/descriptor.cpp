/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/descriptor.hpp"


namespace robo_trace::processing {

Descriptor::Descriptor(const ros::NodeHandle& stage_namespace, const std::string& name) 
: m_name(name){
    
    // Namespace is nested one level down under plugin general ns.
    m_handle = ros::NodeHandle(stage_namespace, name);

}

Descriptor::~Descriptor() = default;


const std::string& Descriptor::getName() const {
    return m_name;
}

ros::NodeHandle& Descriptor::getNodeHandle() {
    return m_handle;
}

}
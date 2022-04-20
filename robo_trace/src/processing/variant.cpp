/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/variant.hpp"


namespace robo_trace::processing {

ProcessingVariant::ProcessingVariant(const std::string& name, const int priority, const std::regex regex, std::vector<Descriptor::Ptr> descriptors) 
: m_name(name), m_priority(priority), m_regex(regex), m_descriptors(descriptors) {
    // 
}

ProcessingVariant::~ProcessingVariant() = default;

const std::string& ProcessingVariant::getName() const {
    return m_name;
}

const int ProcessingVariant::getPriority() const {
    return m_priority;
}

const std::regex ProcessingVariant::getRegex() const {
    return m_regex;
}

const std::vector<Descriptor::Ptr> ProcessingVariant::getDescriptors() const {
    return m_descriptors;
}

}
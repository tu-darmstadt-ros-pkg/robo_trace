/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once

// Std
#include <string>
#include <vector>
#include <memory>
#include <regex>
// Project
#include "robo_trace/processing/descriptor.hpp"


namespace robo_trace::processing {

class ProcessingVariant final {

public:

    typedef std::shared_ptr<ProcessingVariant> Ptr;
    typedef std::shared_ptr<const ProcessingVariant> ConstPtr;

public:

    /**
     * 
     */
    ProcessingVariant(const std::string& name, const int priority, const std::regex regex, std::vector<Descriptor::Ptr> descriptors);

    /**
     * 
     */
    ~ProcessingVariant();

    /**
     * 
     */
    const std::string& getName() const;

    /**
     * 
     */
    const int getPriority() const;

    /**
     * 
     */
    const std::regex getRegex() const;

    /**
     * 
     */
    const std::vector<Descriptor::Ptr> getDescriptors() const;

private:

    /** */
    const std::string m_name;
    
    /** */
    const int m_priority;
    /** */
    const std::regex m_regex;

    /** */
    const std::vector<Descriptor::Ptr> m_descriptors;

};   

}
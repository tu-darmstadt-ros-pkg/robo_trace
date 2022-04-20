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
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/storage/options.hpp"
#include "robo_trace/processing/descriptor.hpp"
#include "robo_trace/processing/modules/downsampling/configuration.hpp"


namespace robo_trace::processing {

class DownsamplingModuleDescriptor final : public Descriptor {

public:

    /**
     * 
     */
    DownsamplingModuleDescriptor(const ros::NodeHandle& stages_namespace);

    /**
     * 
     */
    virtual ~DownsamplingModuleDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const Mode mode) const final override;
    
    /**
     * 
     */
    virtual std::optional<Processor::Ptr> getStage(const robo_trace::store::Container::Ptr& chain_metadata, const Mode mode) final override;

private:

    /** */
    std::vector<DownsamplingMode::Ptr> m_downsampling_modes;

};

}
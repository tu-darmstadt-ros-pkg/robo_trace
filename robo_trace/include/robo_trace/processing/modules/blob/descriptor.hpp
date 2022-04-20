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
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/processing/stage/descriptor.hpp"
#include "robo_trace/storage/options.hpp"


namespace robo_trace {

class BlobDecouplingStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * 
     */
    BlobDecouplingStageDescriptor(const ConnectionOptions::ConstPtr& options, const ros::NodeHandle& stages_namespace);

    /**
     * 
     */
    virtual ~BlobDecouplingStageDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const ProcessingMode mode) const final override;
    
    /**
     * 
     */
    virtual std::optional<ProcessingStage::Ptr> getStage(const DataContainer::Ptr& chain_metadata, const ProcessingMode mode) final override;

private:

    /** */
    const ConnectionOptions::ConstPtr m_connector_options;

};

}
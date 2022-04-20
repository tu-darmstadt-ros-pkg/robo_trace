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
#include <memory>
// Project
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::processing {

class TimeDownsamplingForwardProcessor final : public Processor { 

public:

    /**
     * 
     */
    TimeDownsamplingForwardProcessor(const ros::Duration& m_capture_period);
  
    /**
     * 
     */
    virtual ~TimeDownsamplingForwardProcessor();
    
    /**
     *
     */
    virtual Mode getMode() const final override;
    
    /**
     * 
     */
    virtual void process(const Context::Ptr& context) final override;

private:

    /** */
    const ros::Duration m_capture_period;
    /** */
    ros::Time m_capture_next_time;

};

}
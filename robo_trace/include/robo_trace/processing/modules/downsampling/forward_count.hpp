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

class CountDownsamplingForwardProcessor final : public Processor { 

public:

    /**
     * 
     */
    CountDownsamplingForwardProcessor(const uint32_t leave_out_count);
  
    /**
     * 
     */
    virtual ~CountDownsamplingForwardProcessor();
    
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
    const uint32_t m_leave_out_count;
    /** */
    uint32_t m_message_count;

};

}
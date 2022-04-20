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
#include <memory>
// Project
#include "robo_trace/processing/processor.hpp"
#include "robo_trace/processing/descriptor.hpp"


namespace robo_trace::processing {

/**
 *
 */
class PerformanceMeasuringProcessorDelegate final : public Processor {

public:

    /**
     * TODO
     */
    PerformanceMeasuringProcessorDelegate(const Descriptor::Ptr delegate_descriptor, const Processor::Ptr delegate);

    /**
     * TODO
     */
    virtual ~PerformanceMeasuringProcessorDelegate();
  
    /**
     *
     */
    virtual Mode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const Context::Ptr& context) final override;

private:

    /** */
    const Processor::Ptr m_delegate;  
    /** */
    const Descriptor::Ptr m_delegate_descriptor;

};

} 
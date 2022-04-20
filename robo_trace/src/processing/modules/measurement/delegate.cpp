/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/modules/measurement/delegate.hpp"
// Std
#include <chrono>
// Project
#include "robo_trace/storage/container.hpp"


namespace robo_trace::processing {

PerformanceMeasuringProcessorDelegate::PerformanceMeasuringProcessorDelegate(const Descriptor::Ptr delegate_descriptor, const Processor::Ptr delegate) 
: m_delegate(delegate), 
  m_delegate_descriptor(delegate_descriptor) {
    //
} 
   
PerformanceMeasuringProcessorDelegate::~PerformanceMeasuringProcessorDelegate() = default;


Mode PerformanceMeasuringProcessorDelegate::getMode() const {
    return m_delegate->getMode();
}

void PerformanceMeasuringProcessorDelegate::process(const Context::Ptr& context) {
    
    const std::chrono::steady_clock::time_point pre_invoke_time = std::chrono::steady_clock::now();
    m_delegate->process(context);
    const std::chrono::steady_clock::time_point post_invoke_time = std::chrono::steady_clock::now();

    const unsigned int delay = std::chrono::duration_cast<std::chrono::microseconds>(post_invoke_time - pre_invoke_time).count();

    const robo_trace::store::Container::Ptr& store = context->getMetadata()->getContainer("timing");
    store->append(m_delegate_descriptor->getName(), static_cast<int>(delay));

}

}
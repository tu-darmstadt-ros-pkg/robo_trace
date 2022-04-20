/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Project
#include "robo_trace/processing/modules/downsampling/forward_count.hpp"
// Std
#include <mutex>


namespace robo_trace::processing {

CountDownsamplingForwardProcessor::CountDownsamplingForwardProcessor(const uint32_t leave_out_count) 
: m_leave_out_count(leave_out_count),
  m_message_count(0) {
   //
}

CountDownsamplingForwardProcessor::~CountDownsamplingForwardProcessor() = default;

Mode CountDownsamplingForwardProcessor::getMode() const {
    return Mode::CAPTURE;
}

void CountDownsamplingForwardProcessor::process(const Context::Ptr& context) {

    m_message_count += 1;

    if (m_message_count < m_leave_out_count) {
        context->setTerminated();
    } else {
        m_message_count = 0;
    }

}


}
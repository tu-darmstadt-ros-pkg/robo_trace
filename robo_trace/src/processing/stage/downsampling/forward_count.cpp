// Project
#include "robo_trace/processing/stage/downsampling/forward_count.hpp"
// Std
#include <mutex>


namespace robo_trace {

CountDownsamplingForwardStage::CountDownsamplingForwardStage(const uint32_t leave_out_count) 
: m_leave_out_count(leave_out_count),
  m_message_count(0) {
   //
}

CountDownsamplingForwardStage::~CountDownsamplingForwardStage() = default;

ProcessingMode CountDownsamplingForwardStage::getMode() const {
    return ProcessingMode::CAPTURE;
}

void CountDownsamplingForwardStage::process(const ProcessingContext::Ptr& context) {

    m_message_count += 1;

    if (m_message_count < m_leave_out_count) {
        context->setTerminated();
    } else {
        m_message_count = 0;
    }

}


}
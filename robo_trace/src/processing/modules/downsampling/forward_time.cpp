// Project
#include "robo_trace/processing/modules/downsampling/forward_time.hpp"
// Std
#include <mutex>


namespace robo_trace::processing {

TimeDownsamplingForwardProcessor::TimeDownsamplingForwardProcessor(const ros::Duration& capture_period) 
: m_capture_period(capture_period),
  m_capture_next_time(0) {
   //
}

TimeDownsamplingForwardProcessor::~TimeDownsamplingForwardProcessor() = default;

Mode TimeDownsamplingForwardProcessor::getMode() const {
    return Mode::CAPTURE;
}

void TimeDownsamplingForwardProcessor::process(const Context::Ptr& context) {

    const ros::Time time_now = ros::Time::now();

    if (time_now < m_capture_next_time) {
        context->setTerminated();
    } else {
        m_capture_next_time = time_now + m_capture_period;
    }
    
}


}
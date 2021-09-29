// Project
#include "robo_trace/processing/stage/downsampling/forward_time.hpp"
// Std
#include <mutex>


namespace robo_trace {

TimeDownsamplingForwardStage::TimeDownsamplingForwardStage(const ros::Duration& capture_period) 
: m_capture_period(capture_period),
  m_capture_next_time(0) {
   //
}

TimeDownsamplingForwardStage::~TimeDownsamplingForwardStage() = default;

ProcessingMode TimeDownsamplingForwardStage::getMode() const {
    return ProcessingMode::CAPTURE;
}

void TimeDownsamplingForwardStage::process(const ProcessingContext::Ptr& context) {

    const ros::Time time_now = ros::Time::now();

    if (time_now < m_capture_next_time) {
        context->setTerminated();
    } else {
        m_capture_next_time = time_now + m_capture_period;
    }
    
}


}
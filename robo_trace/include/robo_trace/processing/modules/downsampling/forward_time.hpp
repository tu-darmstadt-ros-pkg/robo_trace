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
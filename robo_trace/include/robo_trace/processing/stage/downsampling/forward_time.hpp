#pragma once

// Std
#include <string>
#include <memory>
// Mongo
#include "robo_trace/config.h"
#include <mongo/client/dbclient.h>
// Project
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/stage/stage.hpp"


namespace robo_trace {

class TimeDownsamplingForwardStage final : public ProcessingStage { 

public:

    /**
     * 
     */
    TimeDownsamplingForwardStage(const ros::Duration& m_capture_period);
  
    /**
     * 
     */
    virtual ~TimeDownsamplingForwardStage();
    
    /**
     *
     */
    virtual ProcessingMode getMode() const final override;
    
    /**
     * 
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /** */
    const ros::Duration m_capture_period;
    /** */
    ros::Time m_capture_next_time;

};

}
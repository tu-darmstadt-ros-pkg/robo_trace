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

class CountDownsamplingForwardStage final : public ProcessingStage { 

public:

    /**
     * 
     */
    CountDownsamplingForwardStage(const uint32_t leave_out_count);
  
    /**
     * 
     */
    virtual ~CountDownsamplingForwardStage();
    
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
    const uint32_t m_leave_out_count;
    /** */
    uint32_t m_message_count;

};

}
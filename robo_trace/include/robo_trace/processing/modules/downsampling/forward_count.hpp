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
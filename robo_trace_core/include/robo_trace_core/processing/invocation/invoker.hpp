#pragma once

// Std
#include <memory>
// Project
#include "robo_trace_plugin_interface/processing/context.hpp"
#include "robo_trace_plugin_interface/processing/stage.hpp"


namespace robo_trace {

/**
 * Manages the invocation of processing stages. The corresponding method in the  
 * processing stages itself *may only* be invoked by a single instance of this 
 * class type!
 */
class ProcessingStageInvoker {

public:

    typedef std::shared_ptr<ProcessingStageInvoker> Ptr;
    typedef std::shared_ptr<const ProcessingStageInvoker> ConstPtr;

public:

    /**
     * 
     */
    ProcessingStageInvoker() = default;

    /**
     * 
     */
    virtual ~ProcessingStageInvoker() = default;

    /**
     * Enqueue the provided message context for processing.
     * 
     * @param context the message context to process
     */
    virtual void enqueue(MessageProcessingContext::Ptr& context) = 0;
 
};

}
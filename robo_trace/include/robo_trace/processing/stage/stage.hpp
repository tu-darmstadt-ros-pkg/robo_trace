#pragma once

// Std
#include <string>
#include <memory>
// Project
#include "robo_trace/modes/modes.hpp"
#include "robo_trace/processing/context.hpp"


namespace robo_trace {

class ProcessingStage {

public:



public:

    typedef std::shared_ptr<ProcessingStage> Ptr;
    typedef std::shared_ptr<const ProcessingStage> ConstPtr;

public:

    /**
     * Constructs a new processing stage.
     */
    ProcessingStage();
  
    /**
     * Destructs this instance.
     */
    virtual ~ProcessingStage();

    /**
     * 
     */
    virtual ProcessingMode getMode() const = 0;

    /**
     * Implements the processing step realized by this stage on
     * the provided context.
     * 
     * @param context the message context to process.
     */
    virtual void process(const ProcessingContext::Ptr& context) = 0;

};

}
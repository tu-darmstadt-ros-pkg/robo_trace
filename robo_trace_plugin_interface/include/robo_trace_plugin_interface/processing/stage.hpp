#pragma once

// Std
#include <string>
#include <memory>
// Project
#include "robo_trace_plugin_interface/processing/context.hpp"


namespace robo_trace {

// Forward declaration
class ProcessingStageDescriptor;


class ProcessingStage {

public:

    /**
     * 
     */
    enum Mode {
        /** */
        FORWARD,
        /** */
        BACKWARD,
        /** */
        VALIDATE
    };

    typedef std::shared_ptr<ProcessingStage> Ptr;
    typedef std::shared_ptr<const ProcessingStage> ConstPtr;

public:

    /**
     * Constructs a new processing stage.
     * 
     * @param mode the processing mode of this stage.
     * @param name the name of this processing stage.
     */
    ProcessingStage(const ProcessingStage::Mode mode, const std::string name);
  
    /**
     * Destructs this instance.
     */
    virtual ~ProcessingStage();

    /**
     * Provides a short, descriptive name of this processing stage.
     * 
     * @return the name of this processing stage.
     */
    const std::string& getName() const;

    /**
     * 
     */
    const ProcessingStage::Mode getMode() const;

    /**
     * Implements the processing step realized by this stage on
     * the provided context.
     * 
     * @param context the message context to process.
     */
    virtual void process(MessageProcessingContext::Ptr& context) = 0;


private:

    /** */
    const ProcessingStage::Mode m_mode;
    /** */
    const std::string m_name;

};

}
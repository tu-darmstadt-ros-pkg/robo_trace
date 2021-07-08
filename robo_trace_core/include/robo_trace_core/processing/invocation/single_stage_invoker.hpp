#pragma once
// Std
#include <memory>
// Project
#include "robo_trace_core/processing/invocation/invoker.hpp"
#include "robo_trace_plugin_interface/processing/stage/processing_stage.hpp"


namespace robo_trace {

/**
 * Manages the invocation of a single processing stage. The corresponding
 * method in the processing stage itself *may only* be invoked by a single
 * instance of this class type!
 */
class SingleProcessingStageInvoker : public ProcessingStageInvoker {

public:

    /**
     * 
     */
    SingleProcessingStageInvoker(const ProcessingStage::Ptr stage);

    /**
     * 
     */
    virtual ~SingleProcessingStageInvoker();

    /**
     * 
     */
    const ProcessingStage::Ptr& stage() const;

protected:
    
    /** */
    const ProcessingStage::Ptr processing_stage;

};

}
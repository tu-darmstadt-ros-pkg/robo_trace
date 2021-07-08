// Base
#include "robo_trace_core/processing/invocation/single_stage_invoker.hpp"


namespace robo_trace {

SingleProcessingStageInvoker(const ProcessingStage::Ptr stage) 
: ProcessingStageInvoker(), processing_stage(stage) {
    //
}

SingleProcessingStageInvoker::~ProcessingStageInvoker() = default;


const ProcessingStage::Ptr& SingleProcessingStageInvoker::stage() const {
    return processing_stage;
}

}
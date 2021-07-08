// Base
#include "robo_trace_core/processing/invocation/chaining_invoker.hpp"


namespace robo_trace {
  
ChainingProcessingStageInvoker::ChainingProcessingStageInvoker(const ProcessingStage::Ptr stage)
: SingleProcessingStageInvoker(stage) {
    //
}  
ChainingProcessingStageInvoker::ChainingProcessingStageInvoker(const ProcessingStage::Ptr stage, const ProcessingStageInvoker::Ptr next_stage_invoker)
: SingleProcessingStageInvoker(stage), next_stage_invoker(next_stage_invoker) {
    //
}

ChainingProcessingStageInvoker::~ChainingProcessingStageInvoker() = delete;


const ProcessingStageInvoker::Ptr& next() const {
    return next_stage_invoker;
}

void ProcessingStageInvoker::next(const ProcessingStageInvoker::Ptr next_stage_invoker) {
    this->next_stage_invoker = next_stage_invoker;
}

void ChainingProcessingStageInvoker::enqueue(const MessageProcessingContext::Ptr &context) {
    processing_stage->process(context);
    // Assumption: The next stage must have been set at this point!!
    next_stage_invoker->enqueue(context);
}

}

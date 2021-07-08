// Base
#include "robo_trace_core/processing/invocation/blocking_invoker.hpp"


namespace robo_trace {

BlockingProcessingStageInvoker::BlockingProcessingStageInvoker(const PipelinProcessingStageHandler &handler) : ProcessingStageInvoker(handler) {
    // 
}

BlockingProcessingStageInvoker::~BlockingProcessingStageInvoker() {
    // 
}


void BlockingProcessingStageInvoker::invoke(const MessageProcessingContext::Ptr &context) {
    invocation_mutex.lock();
    handler.getStage()->onHandleMessage(handler, context);
    invocation_mutex.unlock();
}

}

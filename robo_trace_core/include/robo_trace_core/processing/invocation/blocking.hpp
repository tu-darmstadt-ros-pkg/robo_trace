#pragma once
// Std
#include <mutex>
// Project
#include "robo_trace_plugin_interface/message_processing_context.hpp"
#include "robo_trace_core/processing/invocation/processing_stage_invoker.hpp"


namespace robo_trace {

class BlockingProcessingStageInvoker final : public ProcessingStageInvoker {

public:

    /**
     * 
     */
    BlockingProcessingStageInvoker(const ProcessingStage::Ptr& stage);

    /**
     * 
     */
    virtual ~BlockingProcessingStageInvoker();

    /**
     * 
     */
    virtual void enqueue(const MessageProcessingContext::Ptr &context) final override;

private:

    const std::mutex invocation_mutex;

};

}
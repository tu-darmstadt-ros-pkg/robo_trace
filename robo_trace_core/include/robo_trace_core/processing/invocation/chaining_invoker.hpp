#pragma once
// Std
#include <memory>
// Project
#include "robo_trace_plugin_interface/message_processing_context.hpp"
#include "robo_trace_core/processing/invocation/single_processing_stage_invoker.hpp"


namespace robo_trace {

/**
 * 
 */
class ChainingProcessingStageInvoker final : public SingleProcessingStageInvoker {

public:

    typedef std::shared_ptr<ChainingProcessingStageInvoker> Ptr;
    typedef std::shared_ptr<const ChainingProcessingStageInvoker> ConstPtr;

public:

    /**
     * 
     */
    ChainingProcessingStageInvoker(const ProcessingStage::Ptr stage);

    /**
     * 
     */
    ChainingProcessingStageInvoker(const ProcessingStage::Ptr stage, const ProcessingStageInvoker::Ptr next_stage_invoker);

    /**
     * 
     */
    virtual ~ChainingProcessingStageInvoker();

    /**
     * 
     */
    const ProcessingStageInvoker::Ptr& next() const;

    /**
     * 
     */
    void next(const ProcessingStageInvoker::Ptr next_stage_invoker);

    /**
     * 
     */
    virtual void enqueue(const MessageProcessingContext::Ptr &context) final override;

private:

    /** The next stage to invoke after this one. Maybe a nullptr. */
    ProcessingStageInvoker::Ptr next_stage_invoker;

};

}
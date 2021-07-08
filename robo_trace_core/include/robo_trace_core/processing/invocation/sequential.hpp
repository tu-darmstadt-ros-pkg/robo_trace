#pragma once
// Std
#include <memory>
#include <vector>
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"
#include "robo_trace_plugin_interface/processing/context.hpp"
#include "robo_trace_core/processing/invocation/invoker.hpp"


namespace robo_trace {

/**
 * 
 */
class SequentialProcessingStageInvoker final : public ProcessingStageInvoker {

public:

    /**
     * 
     */
    SequentialProcessingStageInvoker(const std::vector<ProcessingStage::Ptr>& stages);

    /**
     * 
     */
    virtual ~SequentialProcessingStageInvoker();

    /**
     * 
     */
    const std::vector<ProcessingStage::Ptr>& getStages() const;
 
    /**
     * 
     */
    virtual void enqueue(MessageProcessingContext::Ptr& context) final override;

private:

    /** A reference to the ordered set of stages to invoke for any context. */
    const std::vector<ProcessingStage::Ptr>& m_stages;

};

}
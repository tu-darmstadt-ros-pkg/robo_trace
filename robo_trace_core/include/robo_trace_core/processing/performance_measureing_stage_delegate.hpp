#pragma once

// Std
#include <memory>
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"


namespace robo_trace {

/**
 *
 */
class PerformanceMeasuringProcessingStageDelegate final : public ProcessingStage {

public:

    /**
     * TODO
     */
    PerformanceMeasuringProcessingStageDelegate(const ProcessingStage::Ptr delegate);

    /**
     * TODO
     */
    virtual ~PerformanceMeasuringProcessingStageDelegate();

    /**
     * TODO
     */
    const ProcessingStage::Ptr& getDelegate() const; 

    /**
     * TODO
     */
    virtual void process(MessageProcessingContext::Ptr& context) final override;

private:

    const ProcessingStage::Ptr m_delegate;  

};

} 
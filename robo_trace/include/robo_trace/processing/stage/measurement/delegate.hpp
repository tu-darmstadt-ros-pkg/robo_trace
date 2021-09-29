#pragma once

// Std
#include <memory>
// Project
#include "robo_trace/processing/stage/stage.hpp"
#include "robo_trace/processing/stage/descriptor.hpp"


namespace robo_trace {

/**
 *
 */
class PerformanceMeasuringProcessingStageDelegate final : public ProcessingStage {

public:

    /**
     * TODO
     */
    PerformanceMeasuringProcessingStageDelegate(const ProcessingStageDescriptor::Ptr delegate_descriptor, const ProcessingStage::Ptr delegate);

    /**
     * TODO
     */
    virtual ~PerformanceMeasuringProcessingStageDelegate();
  
    /**
     *
     */
    virtual ProcessingMode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /** */
    const ProcessingStage::Ptr m_delegate;  
    /** */
    const ProcessingStageDescriptor::Ptr m_delegate_descriptor;

};

} 
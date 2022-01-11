#pragma once

// Std
#include <memory>
// Project
#include "robo_trace/processing/processor.hpp"
#include "robo_trace/processing/descriptor.hpp"


namespace robo_trace::processing {

/**
 *
 */
class PerformanceMeasuringProcessorDelegate final : public Processor {

public:

    /**
     * TODO
     */
    PerformanceMeasuringProcessorDelegate(const Descriptor::Ptr delegate_descriptor, const Processor::Ptr delegate);

    /**
     * TODO
     */
    virtual ~PerformanceMeasuringProcessorDelegate();
  
    /**
     *
     */
    virtual Mode getMode() const final override;
  
    /**
     * TODO
     */
    virtual void process(const Context::Ptr& context) final override;

private:

    /** */
    const Processor::Ptr m_delegate;  
    /** */
    const Descriptor::Ptr m_delegate_descriptor;

};

} 
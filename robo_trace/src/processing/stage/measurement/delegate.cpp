// Base
#include "robo_trace/processing/stage/measurement/delegate.hpp"
// Std
#include <chrono>
// Project
#include "robo_trace/storage/container.hpp"


namespace robo_trace {

PerformanceMeasuringProcessingStageDelegate::PerformanceMeasuringProcessingStageDelegate(const ProcessingStageDescriptor::Ptr delegate_descriptor, const ProcessingStage::Ptr delegate) 
: m_delegate(delegate), 
  m_delegate_descriptor(delegate_descriptor) {
    //
} 
   
PerformanceMeasuringProcessingStageDelegate::~PerformanceMeasuringProcessingStageDelegate() = default;


ProcessingMode PerformanceMeasuringProcessingStageDelegate::getMode() const {
    return m_delegate->getMode();
}

void PerformanceMeasuringProcessingStageDelegate::process(const ProcessingContext::Ptr& context) {
    
    const std::chrono::steady_clock::time_point pre_invoke_time = std::chrono::steady_clock::now();
    m_delegate->process(context);
    const std::chrono::steady_clock::time_point post_invoke_time = std::chrono::steady_clock::now();

    unsigned int delay = std::chrono::duration_cast<std::chrono::microseconds>(post_invoke_time - pre_invoke_time).count();

    /*
        // TODO: Make the storage keys configurable.
        // Upper level metadata container:
        {   
            // Timing container
            "timing": {
                "stage_xyz": {
                    // In µs
                    "duration": 8194198 
                }
                "stage_abc": {

                }
            }
        }

    */
    const DataContainer::Ptr& store = context->getMetadata()->getContainer("timing");
    store->append(m_delegate_descriptor->getName(), static_cast<int>(delay));

}

}
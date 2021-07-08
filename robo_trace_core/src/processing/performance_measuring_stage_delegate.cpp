// Base
#include "robo_trace_core/processing/performance_measureing_stage_delegate.hpp"
// Std
#include <chrono>
// Project
#include "robo_trace_plugin_interface/processing/metadata.hpp"


namespace robo_trace {

PerformanceMeasuringProcessingStageDelegate::PerformanceMeasuringProcessingStageDelegate(const ProcessingStage::Ptr delegate) 
: ProcessingStage(delegate->getMode(), delegate->getName()), m_delegate(delegate) {
    //
} 
   
PerformanceMeasuringProcessingStageDelegate::~PerformanceMeasuringProcessingStageDelegate() = default;


const ProcessingStage::Ptr& PerformanceMeasuringProcessingStageDelegate::getDelegate() const {
    return m_delegate;
}
    
void PerformanceMeasuringProcessingStageDelegate::process(MessageProcessingContext::Ptr& context) {

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
    const MetadataContainer::Ptr store = context->getMetadata()->getContainer("timing")->getContainer(m_delegate->getName());
    store->append("duration", static_cast<int>(delay));

}

}
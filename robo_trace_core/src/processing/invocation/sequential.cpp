// Base
#include "robo_trace_core/processing/invocation/sequential.hpp"
// Ros
#include "ros/console.h"

namespace robo_trace {

SequentialProcessingStageInvoker::SequentialProcessingStageInvoker(const std::vector<ProcessingStage::Ptr>& stages) 
: m_stages(stages) {
    //
};

SequentialProcessingStageInvoker::~SequentialProcessingStageInvoker() = default;

const std::vector<ProcessingStage::Ptr>& SequentialProcessingStageInvoker::getStages() const {
    return m_stages;
}

void SequentialProcessingStageInvoker::enqueue(MessageProcessingContext::Ptr &context)  {

    for (const ProcessingStage::Ptr& processing_stage : m_stages) {
        //ROS_INFO_STREAM(" - Invoke " << processing_stage->getName());
        processing_stage->process(context);

        if (context->getStatusCode() == MessageProcessingContext::Status::ERROR) {
            //ROS_ERROR_STREAM("Stage " << processing_stage->getName() << " failed due to:" << context->getStatusMessage());
            break;
        }

    }

}

}
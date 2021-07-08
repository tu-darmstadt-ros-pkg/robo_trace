#pragma once
// Std
#include <string>
#include <vector>
#include <memory>
// ROS
#include <ros/ros.h>
// Project
#include "robo_trace_core/processing/forward/descriptor.hpp"
#include "robo_trace_core/processing/forward/pipeline.hpp"


namespace robo_trace {

// Forward reference
class RoboTraceOrchestrator;


class PipelineConstructor {

public:

    /**
     * 
     */
    PipelineConstructor(RoboTraceOrchestrator& orchestrator);

    /**
     * 
     */
    ~PipelineConstructor();

    /**
     * 
     */
    RoboTraceOrchestrator& getOrchestrator();

    /**
     * 
     */
    const std::vector<PipelineDescriptor::Ptr>& getPipelineDescriptors() const;

  
    /**
     * 
     */
    void initialize(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);

    /**
     * 
     */
    ForwardProcessingPipeline::Ptr construct(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type);

private:

    /** */
    RoboTraceOrchestrator& m_orchestrator;

    /** */
    std::vector<PipelineDescriptor::Ptr> m_pipeline_descriptors;

};

}
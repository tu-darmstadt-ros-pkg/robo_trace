// Base
#include "robo_trace_core/processing/forward/constructor.hpp"
// Std
#include <exception>
#include <typeinfo>
#include <stdexcept>
#include <unordered_map>
// Ros
#include "ros/console.h"
// Project
#include "robo_trace_core/persistance/store.hpp"
#include "robo_trace_core/processing/invocation/sequential.hpp"
#include "robo_trace_core/orchestrator.hpp"


namespace robo_trace {

PipelineConstructor::PipelineConstructor(RoboTraceOrchestrator& orchestrator)
: m_orchestrator(orchestrator) {
    // 
}

PipelineConstructor::~PipelineConstructor() = default;


RoboTraceOrchestrator& PipelineConstructor::getOrchestrator() {
    return m_orchestrator;
}

const std::vector<PipelineDescriptor::Ptr>& PipelineConstructor::getPipelineDescriptors() const {
    return m_pipeline_descriptors;
}


void PipelineConstructor::initialize(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle) {

    /*
        Load the descriptors into an name to ptr mapping for easy later retrival.
    */

    std::unordered_map<std::string, ProcessingStageDescriptor::Ptr> descriptors;

    for (const ProcessingStageDescriptor::Ptr& descriptor : m_orchestrator.getProcessingStageDescriptors()) {
        descriptors[descriptor->getName()] = descriptor;
    }

    /*
        Load the pipeline models from the configuration file.
    */

    const ros::NodeHandle& pipeline_ns = ros::NodeHandle(private_node_handle, "pipelines");
    const ros::NodeHandle& pipeline_matching_ns = ros::NodeHandle(pipeline_ns, "matching");
    const ros::NodeHandle& pipeline_sequence_ns = ros::NodeHandle(pipeline_ns, "sequence");

    std::vector<std::string> variant_names;
    pipeline_ns.getParam("versions", variant_names);
    
    for (const std::string& variant_name : variant_names) {
        
        // Should be node_ns/plugin_ns/descriptor_ns/matching/variant_name
        const ros::NodeHandle& variant_matching_ns = ros::NodeHandle(pipeline_matching_ns, variant_name);
      
        int priority;
        variant_matching_ns.getParam("priority", priority);
       
        std::string regex_string;
        variant_matching_ns.getParam("regex", regex_string);

        std::vector<std::string> sequence_names;
        pipeline_sequence_ns.getParam(variant_name, sequence_names);
        
        std::vector<ProcessingStageDescriptor::Ptr> sequence_descriptors;

        for (const std::string& stage_name : sequence_names) {
            
            std::unordered_map<std::string, ProcessingStageDescriptor::Ptr>::iterator matching_descriptors = descriptors.find(stage_name);

            if (matching_descriptors == descriptors.end()) {
                // TODO: Error
                break; // Inner loop
            }

            const ProcessingStageDescriptor::Ptr& descriptor = matching_descriptors->second;
            sequence_descriptors.push_back(descriptor);

        }

        PipelineDescriptor::Ptr pipeline_descriptor = std::make_shared<PipelineDescriptor>(variant_name, priority, std::regex(regex_string), sequence_descriptors);
        m_pipeline_descriptors.push_back(pipeline_descriptor);

    }

}

ForwardProcessingPipeline::Ptr PipelineConstructor::construct(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) {
    
    size_t pipeline_idx = 0;
    
    // The configuration vector is already sored by priority, so we can just iterate 
    // and get the first match.
    for (; pipeline_idx < m_pipeline_descriptors.size(); ++pipeline_idx) {
        if (std::regex_match(topic, m_pipeline_descriptors[pipeline_idx]->getRegex())) {
            break;
        }
    }

    if (pipeline_idx == m_pipeline_descriptors.size()) {
        ROS_INFO_STREAM(" - No matching pipeline definition found!");
        // TODO: Error.
    }

    const PipelineDescriptor::Ptr& descriptor = m_pipeline_descriptors[pipeline_idx];
    ROS_INFO_STREAM(" - Using pipeline model: " << descriptor->getName());

    MessageStore::Ptr message_store = m_orchestrator.getDatabaseConnection()->getStore(topic);
    ForwardProcessingPipeline::Ptr pipeline = std::make_shared<ForwardProcessingPipeline>(m_orchestrator.getBabelFish(), m_orchestrator.getSystemNodeHandle(), message_store, topic);
    
    for (const ProcessingStageDescriptor::Ptr& stage_descriptor : descriptor->getDescriptors()) {
        // The stage for this type and topic.
        const ProcessingStage::Ptr stage = stage_descriptor->getStage(mode, topic, message_type);
        pipeline->getStages().push_back(stage);
        ROS_INFO_STREAM("    + Adding stage: " << stage->getName());
    }

    ROS_INFO_STREAM(" - Using sequential invocation strategy.");
    std::shared_ptr<SequentialProcessingStageInvoker> invoker = std::make_shared<SequentialProcessingStageInvoker>(pipeline->getStages());
    pipeline->setInvoker(std::static_pointer_cast<ProcessingStageInvoker>(invoker));

    ROS_INFO_STREAM(" - Done.");
    return pipeline;

}



}
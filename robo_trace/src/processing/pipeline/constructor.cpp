// Base
#include "robo_trace/processing/pipeline/constructor.hpp"
// Std
#include <exception>
#include <typeinfo>
#include <stdexcept>
#include <unordered_map>
#include <algorithm>
#include <sstream>
#include <functional>
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/processing/stage/measurement/delegate.hpp"
#include "robo_trace/processing/stage/blob/descriptor.hpp"
#include "robo_trace/processing/stage/marshalling/descriptor.hpp"
#include "robo_trace/processing/stage/downsampling/descriptor.hpp"


namespace robo_trace {


PipelineConstructor::PipelineConstructor() = default;

PipelineConstructor::~PipelineConstructor() = default;


const std::vector<PipelineDescriptor::Ptr>& PipelineConstructor::getPipelineDescriptors() const {
    return m_pipeline_descriptors;
}


void PipelineConstructor::initialize(const ConnectionOptions::Ptr& connection_options, ros::NodeHandle& system_node_handle) {

    /*
        Load the processing plugins.
    */

    ros::NodeHandle plugin_ns = ros::NodeHandle(system_node_handle, "plugins");
    ros::NodeHandle stage_ns = ros::NodeHandle(system_node_handle, "stages");
    
    m_processing_plugin_loader = std::make_unique<pluginlib::ClassLoader<RoboTraceProcessingPlugin>>("robo_trace", "robo_trace::RoboTraceProcessingPlugin");

    std::unordered_map<std::string, ProcessingStageDescriptor::Ptr> descriptors;

    if (system_node_handle.hasParam("plugins")) {

        XmlRpc::XmlRpcValue processing_plugins;
        system_node_handle.getParam("plugins", processing_plugins);

        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it_plugin = processing_plugins.begin(); it_plugin != processing_plugins.end(); ++it_plugin) { 

            const std::string& plugin_name = "robo_trace::" + it_plugin->first;
            const boost::shared_ptr<RoboTraceProcessingPlugin> plugin = m_processing_plugin_loader->createInstance(plugin_name);
            
            plugin->initialize(
                // System node handle
                system_node_handle, 
                // Plugin node handle
                plugin_ns,
                // Stage node handle
                stage_ns
            );
            
            for (const ProcessingStageDescriptor::Ptr& descriptor : plugin->getDescriptors()) {
                descriptors[descriptor->getName()] = descriptor;
            }

            m_processing_plugins.push_back(to_std_ptr<RoboTraceProcessingPlugin>(plugin));

        }

    }

    /*
        Load in the default stages.
    */
   
    ProcessingStageDescriptor::Ptr default_marshaller = std::make_shared<BasicMessageMarshallingStageDescriptor>(stage_ns);
    descriptors[default_marshaller->getName()] = default_marshaller;
    
    ProcessingStageDescriptor::Ptr blob_decoupler = std::make_shared<BlobDecouplingStageDescriptor>(connection_options, stage_ns);
    descriptors[blob_decoupler->getName()] = blob_decoupler;
    
    ProcessingStageDescriptor::Ptr downsampler = std::make_shared<DownsamplingStageDescriptor>(stage_ns);
    descriptors[downsampler->getName()] = downsampler;
   
    /*
        Load the pipeline models from the configuration file.
    */

    if (!system_node_handle.hasParam("pipelines")) {

        const std::vector<ProcessingStageDescriptor::Ptr> sequence_descriptors = {default_marshaller};

        m_pipeline_descriptors.push_back(std::make_shared<PipelineDescriptor>(
            // name
            "basic",
            // priority 
            0, 
            // regex
            std::regex("*"), 
            // stages
            sequence_descriptors
        ));
            
    } else {

        XmlRpc::XmlRpcValue pipeline_models;
        system_node_handle.getParam("pipelines", pipeline_models);

        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it_model = pipeline_models.begin(); it_model != pipeline_models.end(); ++it_model) { 

            const std::string& pipeline_model_name = it_model->first;
            const XmlRpc::XmlRpcValue& pipeline_model =  it_model->second;
            
            if (pipeline_model.getType() != XmlRpc::XmlRpcValue::Type::TypeStruct) {
                throw std::runtime_error("Failed loading pipeline models. Unexpected syntax.");
            }

            int priority = pipeline_model["priority"];
            std::string regex_string = pipeline_model["regex"];

            const XmlRpc::XmlRpcValue& sequence_names = pipeline_model["sequence"];
            std::vector<ProcessingStageDescriptor::Ptr> sequence_descriptors;

            for (size_t idx = 0; idx < sequence_names.size(); ++idx) {
                
                const std::string& stage_name = sequence_names[idx];
                
                std::unordered_map<std::string, ProcessingStageDescriptor::Ptr>::iterator matching_descriptors = descriptors.find(stage_name);

                if (matching_descriptors == descriptors.end()) {
                    throw std::runtime_error("Failed finding stage with name '" + stage_name + "'.");
                }

                const ProcessingStageDescriptor::Ptr& descriptor = matching_descriptors->second;
                sequence_descriptors.push_back(descriptor);
    
            }

            PipelineDescriptor::Ptr pipeline_descriptor = std::make_shared<PipelineDescriptor>(pipeline_model_name, priority, std::regex(regex_string), sequence_descriptors);
            m_pipeline_descriptors.push_back(pipeline_descriptor);
            
        }

        std::sort(m_pipeline_descriptors.begin(), m_pipeline_descriptors.end(), 
            [](const PipelineDescriptor::Ptr& one, const PipelineDescriptor::Ptr& two) -> bool { 
                return one->getPriority() > two->getPriority(); 
            }
        );


    }

}

std::vector<ProcessingStage::Ptr> PipelineConstructor::construct(const ProcessingMode mode, const DataContainer::Ptr& chain_data, const std::string& topic) {
    
    /*
        Fetch the corresponding pipeline descriptor.
    */

    size_t pipeline_idx = 0;
    
    // The configuration vector is already sored by priority, so we can just iterate 
    // and get the first match.
    for (; pipeline_idx < m_pipeline_descriptors.size(); ++pipeline_idx) {
        if (std::regex_match(topic, m_pipeline_descriptors[pipeline_idx]->getRegex())) {
            break;
        }
    }

    if (pipeline_idx == m_pipeline_descriptors.size()) {
        throw std::runtime_error("Failed finding matching pipeline model for topic '" + topic + "'!");
    }

    const PipelineDescriptor::Ptr& descriptor = m_pipeline_descriptors[pipeline_idx];

    /*
        Build the pipeline.
    */

    std::vector<ProcessingStage::Ptr> pipeline;
    
    for (const ProcessingStageDescriptor::Ptr& stage_descriptor : descriptor->getDescriptors()) {

        if (!stage_descriptor->isModeSupported(mode)) {
            continue;
        }

        // The stage for this type and topic.
        const std::optional<ProcessingStage::Ptr> o_stage = stage_descriptor->getStage(chain_data, mode);

        if (!o_stage) {
            continue;
        }

        const ProcessingStage::Ptr stage = o_stage.value();
        // Measure execution time.
        const PerformanceMeasuringProcessingStageDelegate::Ptr measurement_delegate = std::make_shared<PerformanceMeasuringProcessingStageDelegate>(stage_descriptor, stage);
        pipeline.push_back(measurement_delegate);

    }

    /*
        Mode specific preprocessing.
    */

    switch (mode) {
        
        case ProcessingMode::REPLAY:
        case ProcessingMode::VALIDATE: 
            std::reverse(std::begin(pipeline), std::end(pipeline));
            break;

        case ProcessingMode::CAPTURE:
        default:
            break;

    }
   
    return pipeline;

}



}
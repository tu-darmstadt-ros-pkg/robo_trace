/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/constructor.hpp"
// Std
#include <exception>
#include <typeinfo>
#include <stdexcept>
#include <unordered_map>
#include <algorithm>
#include <sstream>
#include <functional>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/processing/modules/measurement/delegate.hpp"
#include "robo_trace/processing/modules/marshalling/descriptor.hpp"
#include "robo_trace/processing/modules/downsampling/descriptor.hpp"

#include <iostream>

namespace robo_trace::processing {


Constructor::Constructor() = default;

Constructor::~Constructor() {

    // Unload all the plugins explicitly.
    m_processing_plugins.clear();
    // And the variant descriptions as they hold pointers to the descriptors.
    m_pipeline_descriptors.clear();

    /*
        Note: All pointer to pluginlib loaded classes should(/must) have been freed at this point 
              or it won't be possible to unload the classes propperly.
    */

}


const std::vector<ProcessingVariant::Ptr>& Constructor::getProcessingVariants() const {
    return m_pipeline_descriptors;
}


void Constructor::initialize(const robo_trace::store::Options::Ptr& connection_options, ros::NodeHandle& system_node_handle) {

    /*
        Load the processing plugins.
    */

    ros::NodeHandle plugin_ns = ros::NodeHandle(system_node_handle, "plugins");
    ros::NodeHandle stage_ns = ros::NodeHandle(system_node_handle, "stages");
    
    m_processing_plugin_loader = std::make_unique<pluginlib::ClassLoader<Plugin>>("robo_trace", "robo_trace::processing::Plugin");

    std::unordered_map<std::string, Descriptor::Ptr> descriptors;

    if (system_node_handle.hasParam("plugins")) {

        XmlRpc::XmlRpcValue processing_plugins;
        system_node_handle.getParam("plugins", processing_plugins);

        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it_plugin = processing_plugins.begin(); it_plugin != processing_plugins.end(); ++it_plugin) { 
            
            const std::string& plugin_name = it_plugin->first;
            const std::string namespace_param_key = ros::names::append(plugin_name, "namespace");
            
            std::string plugin_path; 
            
            if (plugin_ns.hasParam(namespace_param_key)) {
                
                std::string plugin_namespace;
                plugin_ns.getParam(namespace_param_key, plugin_namespace);
                
                plugin_path = plugin_namespace + "::" + plugin_name;

            } else {
                plugin_path = plugin_name;
            }

             
            ROS_INFO_STREAM("Loading: " << plugin_path);
            const boost::shared_ptr<Plugin> plugin = m_processing_plugin_loader->createInstance(plugin_path);
            
            plugin->initialize(
                // System node handle
                system_node_handle, 
                // Plugin node handle
                plugin_ns,
                // Stage node handle
                stage_ns
            );
            
            for (const Descriptor::Ptr& descriptor : plugin->getModules()) {
                descriptors[descriptor->getName()] = descriptor;
            }

            m_processing_plugins.push_back(to_std_ptr<Plugin>(plugin));

        }

    }

    /*
        Load in the default stages.
    */
  
    Descriptor::Ptr default_marshaller = std::make_shared<BasicMarshallingModuleDescriptor>(stage_ns);
    descriptors[default_marshaller->getName()] = default_marshaller;
 
    Descriptor::Ptr downsampler = std::make_shared<DownsamplingModuleDescriptor>(stage_ns);
    descriptors[downsampler->getName()] = downsampler;
    
    /*
        Load the pipeline models from the configuration file.
    */

    if (!system_node_handle.hasParam("pipelines")) {

        const std::vector<Descriptor::Ptr> sequence_descriptors = {default_marshaller};

        m_pipeline_descriptors.push_back(std::make_shared<ProcessingVariant>(
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
            std::vector<Descriptor::Ptr> sequence_descriptors;

            for (size_t idx = 0; idx < sequence_names.size(); ++idx) {
                
                const std::string& stage_name = sequence_names[idx];
               
                std::unordered_map<std::string, Descriptor::Ptr>::iterator matching_descriptors = descriptors.find(stage_name);

                if (matching_descriptors == descriptors.end()) {
                    throw std::runtime_error("Failed finding stage with name '" + stage_name + "'.");
                }

                const Descriptor::Ptr& descriptor = matching_descriptors->second;
                sequence_descriptors.push_back(descriptor);
    
            }

            const ProcessingVariant::Ptr pipeline_descriptor = std::make_shared<ProcessingVariant>(pipeline_model_name, priority, std::regex(regex_string), sequence_descriptors);
            m_pipeline_descriptors.push_back(pipeline_descriptor);
            
        }

        std::sort(m_pipeline_descriptors.begin(), m_pipeline_descriptors.end(), 
            [](const ProcessingVariant::Ptr& one, const ProcessingVariant::Ptr& two) -> bool { 
                return one->getPriority() > two->getPriority(); 
            }
        );


    }

}

std::vector<Processor::Ptr> Constructor::construct(const Mode mode, const robo_trace::store::Container::Ptr& chain_data, const std::string& topic) {
    
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

    const ProcessingVariant::Ptr& descriptor = m_pipeline_descriptors[pipeline_idx];
    ROS_INFO_STREAM(" - " << descriptor->getName() << " (Topic: " << topic << ")");
    
    /*
        Build the pipeline.
    */

    std::vector<Processor::Ptr> pipeline;
    
    for (const Descriptor::Ptr& stage_descriptor : descriptor->getDescriptors()) {
       
        if (!stage_descriptor->isModeSupported(mode)) {
            continue;
        }
       
        // The stage for this type and topic.
        const std::optional<Processor::Ptr> o_stage = stage_descriptor->getStage(chain_data, mode);

        if (!o_stage) {
            continue;
        }

        const Processor::Ptr stage = o_stage.value();

#ifdef EVALUATION_CAPTURE_PIPELINE_TIMINGS
        if (mode == Mode::CAPTURE) {
            pipeline.push_back(std::make_shared<PerformanceMeasuringProcessorDelegate>(stage_descriptor, stage));
        } else {  
            pipeline.push_back(stage);  
        }  
#else
        pipeline.push_back(stage);
#endif 
       
    }

    /*
        Mode specific preprocessing.
    */

    switch (mode) {
        
        case Mode::REPLAY:
        case Mode::VALIDATE: 
            std::reverse(std::begin(pipeline), std::end(pipeline));
            break;

        case Mode::CAPTURE:
        default:
            break;

    }
   
    return pipeline;

}



}
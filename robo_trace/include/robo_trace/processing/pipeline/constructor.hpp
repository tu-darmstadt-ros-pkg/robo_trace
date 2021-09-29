#pragma once

// Std
#include <string>
#include <vector>
#include <memory>
// ROS
#include <ros/ros.h>
// PluginLib
#include <pluginlib/class_loader.h>
// Project
#include "robo_trace/processing/plugin/plugin.hpp"
#include "robo_trace/processing/pipeline/descriptor.hpp"
#include "robo_trace/storage/options.hpp"


namespace robo_trace {

class PipelineConstructor {

public:

    typedef std::shared_ptr<PipelineConstructor> Ptr;
    typedef std::shared_ptr<const PipelineConstructor> ConstPtr;

public:

    /**
     * 
     */
    PipelineConstructor();

    /**
     * 
     */
    ~PipelineConstructor();

    /**
     * 
     */
    const std::vector<PipelineDescriptor::Ptr>& getPipelineDescriptors() const;

  
    /**
     * 
     */
    void initialize(const ConnectionOptions::Ptr& connection_options, ros::NodeHandle& system_node_handle);

    /**
     * 
     */
    std::vector<ProcessingStage::Ptr> construct(const ProcessingMode mode, const DataContainer::Ptr& summary, const std::string& topic);

private:

    /** */
    std::vector<PipelineDescriptor::Ptr> m_pipeline_descriptors;

    /** Holds all plugins that provide processing functionality. */
    std::vector<RoboTraceProcessingPlugin::Ptr> m_processing_plugins;
    /** */
    std::unique_ptr<pluginlib::ClassLoader<robo_trace::RoboTraceProcessingPlugin>> m_processing_plugin_loader; 

};

}
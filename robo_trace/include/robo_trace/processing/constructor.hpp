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
#include "robo_trace/storage/options.hpp"
#include "robo_trace/processing/processor.hpp"
#include "robo_trace/processing/descriptor.hpp"
#include "robo_trace/processing/plugin.hpp"
#include "robo_trace/processing/variant.hpp"


namespace robo_trace::processing {

class Constructor {

public:

    typedef std::shared_ptr<Constructor> Ptr;
    typedef std::shared_ptr<const Constructor> ConstPtr;

public:

    /**
     * 
     */
    Constructor();

    /**
     * 
     */
    ~Constructor();

    /**
     * 
     */
    const std::vector<ProcessingVariant::Ptr>& getProcessingVariants() const;

    /**
     * 
     */
    void initialize(const robo_trace::store::Options::Ptr& connection_options, ros::NodeHandle& system_node_handle);

    /**
     * 
     */
    std::vector<Processor::Ptr> construct(const Mode mode, const robo_trace::store::Container::Ptr& summary, const std::string& topic);

private:

    /** */
    std::vector<ProcessingVariant::Ptr> m_pipeline_descriptors;

    /** Holds all plugins that provide processing functionality. */
    std::vector<Plugin::Ptr> m_processing_plugins;
    /** */
    std::unique_ptr<pluginlib::ClassLoader<Plugin>> m_processing_plugin_loader; 

};

}
#pragma once
// Std
#include <memory>
#include <string>
#include <vector>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace_plugin_interface/processing/descriptor.hpp"


namespace robo_trace {

class RoboTraceProcessingPlugin {

public:

    typedef std::shared_ptr<RoboTraceProcessingPlugin> Ptr;
    typedef std::shared_ptr<const RoboTraceProcessingPlugin> ContPtr;

public:

    /**
     * 
     */
    RoboTraceProcessingPlugin(const std::string& name);

    /**
     * 
     */
    virtual ~RoboTraceProcessingPlugin();

    /**
     * 
     */
    const std::string& getName() const;

    /**
     * 
     */
    const ros::NodeHandle& getSystemNodeHandle() const;

    /**
     * 
     */
    const ros::NodeHandle& getPluginNodeHandle() const;

    /**
     * 
     */
    void initialize(ros::NodeHandle& system_node_handle);

    /**
     * 
     */
    virtual const std::vector<ProcessingStageDescriptor::Ptr>& getDescriptors() const = 0;

protected:

    /**
     * 
     */
    virtual void setup();

protected:

    /** */
    const std::string m_name;

    /** */
    ros::NodeHandle m_system_node_handle;
    /** */
    ros::NodeHandle m_plugin_node_handle;

};

}
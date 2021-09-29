#pragma once

// Std
#include <memory>
#include <string>
#include <vector>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/processing/stage/descriptor.hpp"


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
    const std::vector<ProcessingStageDescriptor::Ptr>& getDescriptors() const;

    /**
     * 
     */
    ros::NodeHandle& getSystemNodeHandle();

    /**
     * 
     */
    ros::NodeHandle& getPluginNodeHandle();

    /**
     * 
     */
    ros::NodeHandle& getStageNodeHandle();

    /**
     * 
     */
    void initialize(ros::NodeHandle& system_node_handle, ros::NodeHandle& plugin_node_handle, ros::NodeHandle& stage_node_handle);   

protected:

    /**
     * 
     */
    virtual std::vector<ProcessingStageDescriptor::Ptr> setup() = 0;

protected:

    /** */
    const std::string m_name;

    /** */
    ros::NodeHandle m_system_node_handle;
    /** */
    ros::NodeHandle m_plugin_node_handle;
    /** */
    ros::NodeHandle m_stage_node_handle;

    /** */
    std::vector<ProcessingStageDescriptor::Ptr> m_descriptors;

};

}
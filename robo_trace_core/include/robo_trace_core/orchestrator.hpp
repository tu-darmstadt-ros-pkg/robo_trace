#pragma once
// Std
#include <memory>
#include <vector>
#include <unordered_map>
// ROS
#include <ros/ros.h>
// PluginLib
#include <pluginlib/class_loader.h>
// ddynamic_reconfigure
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
// BabelFish
#include <ros_babel_fish/babel_fish.h>
// Project
#include "robo_trace_plugin_interface/plugin.hpp"
#include "robo_trace_core/processing/forward/constructor.hpp"
#include "robo_trace_core/processing/forward/pipeline.hpp"
#include "robo_trace_core/persistance/connection.hpp"


namespace robo_trace {

class RoboTraceOrchestrator {

public: 

    /**
     * 
     */
    RoboTraceOrchestrator();

    /**
     * 
     */
    ~RoboTraceOrchestrator();

    /**
     * 
     */
    ros::NodeHandle& getGloablNodeHandle();

    /**
     * 
     */
    ros::NodeHandle& getSystemNodeHandle();

    /**
     * 
     */
    ros_babel_fish::BabelFish& getBabelFish();

    /**
     * 
     */
    MongoDBConnection::Ptr& getDatabaseConnection();

    /**
     * 
     */
    std::vector<RoboTraceProcessingPlugin::Ptr>& getProcessingPlugins();

    /**
     * 
     */
    std::vector<ProcessingStageDescriptor::Ptr>& getProcessingStageDescriptors();

    /**
     * 
     */
    void initialize(ros::NodeHandle &node_handle, ros::NodeHandle &node_handle_private);

private:

    /**
     * 
     */
    void onCheckForNewTopics(const ros::TimerEvent &event);

private:

    
    /** */
    ros::NodeHandle m_global_node_handle;
    /** */
    ros::NodeHandle m_system_node_handle;

    /** */
    ros_babel_fish::BabelFish m_fish;
    /** */
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_reconfigure;

    /** */
    ros::Timer m_check_for_topics_timer;

    /** */
    std::unordered_map<std::string, ForwardProcessingPipeline::Ptr> m_pipelines;

    /** */
    PipelineConstructor m_forward_pipeline_constructor;  
    /** */
    MongoDBConnection::Ptr m_database_connection;

    /** */
    std::vector<ProcessingStageDescriptor::Ptr> m_processing_stage_descriptors;

    /** Holds all plugins that provide processing functionality. */
    std::vector<RoboTraceProcessingPlugin::Ptr> m_processing_plugins;
    /** */
    std::unique_ptr<pluginlib::ClassLoader<robo_trace::RoboTraceProcessingPlugin>> m_processing_plugin_loader; 

};

}
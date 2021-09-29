#pragma once

// ROS
#include <ros/ros.h>
// Project
#include "robo_trace/storage/options.hpp"
#include "robo_trace/processing/pipeline/constructor.hpp"
#include "robo_trace/processing/stage/descriptor.hpp"
#include "robo_trace/modes/capture/options.hpp"
#include "robo_trace/modes/capture/persistor.hpp"


namespace robo_trace {

class RoboTraceRecorder {

public:

    /**
     *
     */
    RoboTraceRecorder(ros::NodeHandle& system_node_handle);

    /**
     *
     */
    ~RoboTraceRecorder();

    /**
     * 
     */
    void initialize(int argc, char** argv);

private:
   
    /**
     * 
     */
    void onCheckForNewTopics(const ros::TimerEvent &event);

    /**
     * 
     */
    void record(const ros::master::TopicInfo& topic_info);

    /** 
     *
     */
    bool isToBeRecorded(const std::string& topic, const bool is_capture_node = false); 

private:

    
    /** */
    RecorderOptions::Ptr m_option_recorder;
    /** */
    ConnectionOptions::Ptr m_options_connection;

    /** */
    ros::NodeHandle m_system_node_handle;
    /** */
    ros::Timer m_check_topics_timer;

    /** */
    PipelineConstructor m_pipeline_constructor;
    /** */
    ProcessingStageDescriptor::Ptr m_storage_stage_descriptor;

    /** */
    std::unordered_map<std::string, TopicPersistor::Ptr> m_persistors;


};

}
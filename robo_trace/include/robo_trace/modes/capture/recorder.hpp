#pragma once

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/options.hpp"
#include "robo_trace/storage/persistor.hpp"
#include "robo_trace/storage/stream.hpp"
#include "robo_trace/processing/constructor.hpp"
#include "robo_trace/processing/descriptor.hpp"
#include "robo_trace/modes/capture/options.hpp"
#include "robo_trace/modes/capture/persistor.hpp"


namespace robo_trace::capture {

class Recorder {

public:

    /**
     *
     */
    Recorder(ros::NodeHandle& system_node_handle);

    /**
     *
     */
    ~Recorder();

    /**
     * 
     */
    void initialize(int argc, char** argv);

    /**
     * 
     */
    void terminate(int signal);

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
    robo_trace::capture::Options::Ptr m_option_recorder;
    /** */
    robo_trace::store::Options::Ptr m_options_connection;

    /** */
    ros::NodeHandle m_system_node_handle;
    /** */
    ros::Timer m_check_topics_timer;

    /** */
    std::unique_ptr<ros::CallbackQueue> m_persistor_callback_queue;
    /** */
    std::unique_ptr<ros::AsyncSpinner> m_persistor_spinner;

    /** */
    robo_trace::processing::Constructor m_pipeline_constructor;
    /** */
    robo_trace::store::Persistor::Ptr m_persistor_metadata;

#ifdef PERSISTOR_USE_UNIQUE_BUCKET
    /** */
    robo_trace::store::StreamHandler::Ptr m_stream_handler;
#endif

    /** */
    std::unordered_map<std::string, MessageStreamRecorder::Ptr> m_persistors;


};

}
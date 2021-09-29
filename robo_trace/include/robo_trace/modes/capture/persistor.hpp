#pragma once

// Std
#include <string>
#include <memory>
// Ros
#include <ros/ros.h>
#include <ros/message_event.h>
// Babel Fish
#include <ros_babel_fish/babel_fish.h>
// Project
#include "robo_trace/processing/stage/stage.hpp"
#include "robo_trace/modes/capture/options.hpp"


namespace robo_trace {

class TopicPersistor {

public:

    typedef std::shared_ptr<TopicPersistor> Ptr;
    typedef std::shared_ptr<const TopicPersistor> ConstPtr;

public:

    /**
     * 
     */
    TopicPersistor(const RecorderOptions::ConstPtr options_recorder, const std::vector<ProcessingStage::Ptr>& pipeline, ros::NodeHandle& node_handle, const std::string& topic);


    /**
     * 
     */
    ~TopicPersistor();
  
    /**
     * 
     */
    const std::string& getTopic() const;
    
    /**
     * 
     */
    const std::vector<ProcessingStage::Ptr>& getPipeline() const;

    /**
     * 
     */
    ros::NodeHandle& getNodeHandle();

    /**
     * 
     */
    void start();

    /**
     * 
     */
    void stop();

    /**
     * 
     * 
     * TODO: Is it feasible to retrieve a non const pointer here? From
     *   the API docs there seems to be such method signature present.
     */
    void process(const ros::MessageEvent<const ros_babel_fish::BabelFishMessage>& event);

private:   

    /** */
    const std::string m_topic;
    /** */
    const RecorderOptions::ConstPtr m_options_recorder;

    /** */
    ros::NodeHandle& m_node_handle;
    /** */
    const std::vector<ProcessingStage::Ptr> m_pipeline;
    
    /** */
    uint32_t m_messages_received_local = 0;
    /** */
    // std::atomic<uint32_t>& m_messages_received_total;

    /** */
    bool m_subscriber_active = false;
    /** */
    ros::Subscriber m_subscriber;


    
};

}
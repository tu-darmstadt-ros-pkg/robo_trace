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
#include "robo_trace/parameters.hpp"
#include "robo_trace/processing/processor.hpp"
#include "robo_trace/modes/capture/options.hpp"


namespace robo_trace::capture {

class TopicPersistor {

public:

    typedef std::shared_ptr<TopicPersistor> Ptr;
    typedef std::shared_ptr<const TopicPersistor> ConstPtr;

public:

    /**
     * 
     */
    TopicPersistor(const Options::ConstPtr options_recorder, const std::vector<robo_trace::processing::Processor::Ptr>& pipeline, ros::NodeHandle& node_handle, const std::string& topic);


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
    const std::vector<robo_trace::processing::Processor::Ptr>& getPipeline() const;

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
    const Options::ConstPtr m_options_recorder;

    /** */
    ros::NodeHandle& m_node_handle;
    /** */
    const std::vector<robo_trace::processing::Processor::Ptr> m_pipeline;
    
    /** */
    uint32_t m_messages_received_local = 0;
    /** */
    // std::atomic<uint32_t>& m_messages_received_total;

    /** */
    bool m_subscriber_active = false;
    /** */
    ros::Subscriber m_subscriber;

#ifdef RECORDING_SIGNAL_PIPELINE_PASS
    ros::Publisher m_publisher_signal_pipeline_pass;
#endif

    
};

}
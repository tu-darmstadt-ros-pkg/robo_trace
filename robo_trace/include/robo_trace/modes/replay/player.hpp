#pragma once

// Std
#include <queue>
// ROS
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/babel_fish.h>
// Project
#include "robo_trace/processing/pipeline/constructor.hpp"
#include "robo_trace/modes/replay/publisher.hpp"
#include "robo_trace/modes/replay/time.hpp"
#include "robo_trace/modes/replay/options.hpp"


namespace robo_trace {

class MessagePublisherComparator {

public:
    
    bool operator() (MessagePublisher::Ptr one, MessagePublisher::Ptr two) {
       return two->getNextPublicationTime().value_or(0) < one->getNextPublicationTime().value_or(0);
    }

};

class RoboTracePlayer {

public:

    /**
     *
     */
    RoboTracePlayer(ros::NodeHandle& system_node_handle);

    /**
     *
     */
    ~RoboTracePlayer();

    /**
     *
     */
    bool isCompleted() const;

    /**
     * 
     */
    void initialize(int argc, char** argv);

    /**
     *
     */
    void play();

private:

    /**
     *
     */
    bool isToBePlayedBack(const std::string& topic) const;

    /**
     *
     */
    void buffer();

    /**
     *
     */
    std::optional<double> getRecordStartTime();

    /**
     *
     */
    std::optional<double> getRecordStartTime(const std::shared_ptr<mongo::DBClientConnection> connection);

private:

    /** */
    PlayerOptions::Ptr m_options_player;
    /** */
    ConnectionOptions::Ptr m_options_connection;

    /** */
    ros::NodeHandle m_system_node_handle;
    /** */
    ros_babel_fish::BabelFish m_babel_fish;

    /** */
    TimeManager m_time_manager;
    /** */
    PipelineConstructor m_pipeline_constructor;

    /** */
    std::vector<MessagePublisher::Ptr> m_message_publishers;
    /** */
    std::priority_queue<MessagePublisher::Ptr, std::vector<MessagePublisher::Ptr>, MessagePublisherComparator> m_publishing_queue;

};

}
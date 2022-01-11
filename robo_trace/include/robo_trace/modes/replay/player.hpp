#pragma once

// Std
#include <string>
#include <memory>
#include <queue>
#include <vector>
#include <optional>
// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
// BabelFish
#include <ros_babel_fish/babel_fish.h>
// Player
#include "robo_trace/processing/constructor.hpp"
#include "robo_trace/storage/options.hpp"
#include "robo_trace/modes/replay/publisher.hpp"
#include "robo_trace/modes/replay/time.hpp"
#include "robo_trace/modes/replay/options.hpp" 

namespace robo_trace::replay {

class MessagePublisherComparator {

public:
    
    bool operator() (MessagePublisher::Ptr one, MessagePublisher::Ptr two) {
       return two->getNextPublicationTime().value_or(0) < one->getNextPublicationTime().value_or(0);
    }

};

class PlayerBase {

private:

    /**
     * 
     */
    enum MessagePosition {
        /** */
        First,
        /** */
        Last
    };

public:

    /**
     * 
     */
    PlayerBase(ros::NodeHandle& system_node_handle);

    /**
     * 
     */
    virtual ~PlayerBase();

    /**
     * 
     */
    void initialize(int argc, char** argv);


protected:

    /**
     * 
     */
    void setup(const std::optional<float> time_start, const std::optional<float> time_end);

     /**
     * 
     */
    void buffer();

protected:

    /**
     * 
     */
    virtual void initialize();

    /**
     *
     */
    virtual bool isToBePlayedBack(const std::string& topic) const;

    /**
     * 
     */
    std::optional<double> getFirstMessageTime();

    /**
     * 
     */
    std::optional<double> getLastMessageTime();

    /**
     * 
     */
    std::optional<double> getRecordingStartTime();

private:

    /**
     * 
     */
    std::optional<double> getMessageTimeLimit(const PlayerBase::MessagePosition& position);

protected:

    /** */
    robo_trace::replay::Options::Ptr m_options_player;
    /** */
    robo_trace::store::Options::Ptr m_options_connection;

    /** */
    ros::NodeHandle m_system_node_handle;
    /** */
    ros_babel_fish::BabelFish m_babel_fish;

    /** */
    std::unique_ptr<ros::CallbackQueue> m_loader_callback_queue;
    /** */
    std::unique_ptr<ros::AsyncSpinner> m_loader_spinner;

    /** */
    TimeManager m_time_manager;
    /** */
    robo_trace::processing::Constructor m_pipeline_constructor;

    /** */
    std::vector<MessagePublisher::Ptr> m_message_publishers;
    /** */
    std::priority_queue<MessagePublisher::Ptr, std::vector<MessagePublisher::Ptr>, MessagePublisherComparator> m_publishing_queue;

};

}
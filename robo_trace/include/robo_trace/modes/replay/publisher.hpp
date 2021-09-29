#pragma once

// Std
#include <string>
#include <utility>
#include <memory>
#include <optional>
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/babel_fish.h>
// Project
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/result.hpp"
#include "robo_trace/modes/replay/loader.hpp"
#include "robo_trace/modes/replay/options.hpp"


namespace robo_trace {

class MessagePublisher {

public:

    typedef std::shared_ptr<MessagePublisher> Ptr;
    typedef std::shared_ptr<const MessagePublisher> ConstPtr; 

public:

    /**
     *
     */
    MessagePublisher(ros_babel_fish::BabelFish& fish, ros::NodeHandle& node_handle, const PlayerOptions::ConstPtr& player_options, const MessageLoader::Ptr& loader, const DataContainer::Ptr& data);

    /**
     *
     */
    ~MessagePublisher();

    /**
     *
     */
    bool isBlocked() const;

    /**
     *
     */
    bool isCompleted() const;

    /**
     *
     */
    std::optional<double> getNextPublicationTime(); 

    /**
     *
     */
    void skip();

    /**
     *
     */
    void skip(uint32_t amount);

    /**
     *
     */
    void publish();

private:

    /**
     *
     */
    void getNextMessage();
  
private:

    /** */
    const MessageLoader::Ptr m_message_loader;
    /** */
    const PlayerOptions::ConstPtr m_player_options;

    /** */
    ros::Publisher m_publisher;

    /** */
    bool m_next_message_valid;
    /** */
    double m_next_message_time;
    /** */
    ros_babel_fish::BabelFishMessage::ConstPtr m_next_message_data;
  

};


}
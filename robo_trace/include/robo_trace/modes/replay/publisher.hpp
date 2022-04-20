/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
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
#include "robo_trace/modes/replay/loader.hpp"


namespace robo_trace::replay {

class MessagePublisher {

public:

    typedef std::shared_ptr<MessagePublisher> Ptr;
    typedef std::shared_ptr<const MessagePublisher> ConstPtr; 

public:

    /**
     *
     */
    MessagePublisher(
        const MessageLoader::Ptr& loader, 
        const robo_trace::store::Container::Ptr& data,
        ros_babel_fish::BabelFish& fish, 
        ros::NodeHandle& node_handle, 
        const std::string topic_prefix,
        const size_t topic_queue_size
    );

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
    void reset(const double time);

    /**
     *
     */
    void publish();

private:

    /**
     *
     */
    void advance();
  
private:

    /** */
    const MessageLoader::Ptr m_message_loader;
    /** */
    ros::Publisher m_message_publisher;
    
    /** */
    bool m_next_message_valid;
    /** */
    double m_next_message_time;
    /** */
    ros_babel_fish::BabelFishMessage::ConstPtr m_next_message_data;
  

};


}
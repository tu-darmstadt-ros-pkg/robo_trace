/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/modes/replay/publisher.hpp"
// Std
#include <functional>
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"


namespace robo_trace::replay {

 
MessagePublisher::MessagePublisher(const MessageLoader::Ptr& loader, const robo_trace::store::Container::Ptr& data, ros_babel_fish::BabelFish& fish, ros::NodeHandle& node_handle, const std::string topic_prefix, const size_t topic_queue_size) 
: m_message_loader(loader), m_next_message_valid(false) {

    /*
        Create a publisher for the topic.
    */

    const std::string topic = topic_prefix + data->getString("topic");
    const std::string message_type = data->getString("message_type");

    // TODO: Implement latching.
    m_message_publisher = fish.advertise(node_handle, message_type, topic, topic_queue_size, true);
    
}   

MessagePublisher::~MessagePublisher() = default;

bool MessagePublisher::isBlocked() const {

    // A message is cached for publication.
    if (m_next_message_valid) {
        return false;
    }

    // The loader is completed. We are not blocked, but finished.
    if (m_message_loader->isCompleted()) {
        return false;
    }

    // The loader maybe loading.
    return !m_message_loader->isValid();
   
}

bool MessagePublisher::isCompleted() const {
    return !m_next_message_valid && m_message_loader->isCompleted();
}
  
std::optional<double> MessagePublisher::getNextPublicationTime() {
    
    advance();
    
    if (m_next_message_valid) {
        return m_next_message_time;
    } else {
        return {};
    }

}

void MessagePublisher::skip() {
    skip(1);
}

void MessagePublisher::skip(uint32_t amount) {

    if (amount == 0) {
        return;
    }

    m_next_message_valid = false; 

    if (isCompleted()) {
        return;
    }

    // We are doing one less iteration. In this loop we are just trowing away messages. 
    // After that we need to save the message again.
    for (uint32_t idx = 1; idx < amount; ++idx) {

        const std::optional<std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr>> optional_next_message = m_message_loader->next();

        if (!optional_next_message.has_value()) {
            break;
        }
        
    }

    advance();

}

void MessagePublisher::reset(const double time) {
    m_next_message_valid = false; 
    m_message_loader->reset(time);
}
  
void MessagePublisher::publish() {

    // Fetch the next message if we haven't already (should actually never happen, because publish is invoked on the correct time only).
    advance();

    // No more messages available 
    if (!m_next_message_valid) { 
        return;
    }
    
    ROS_INFO_STREAM("Sending message to " << m_message_publisher.getTopic());

    m_message_publisher.publish(m_next_message_data);
    m_next_message_valid = false;
    
    // Prepare the next message.
    advance();

}
    
void MessagePublisher::advance() {
   
    if (m_next_message_valid) {
        return;
    }

    if (m_message_loader->isCompleted()) {
        m_next_message_valid = false;
    } else {
        
        // TODO: This will return an empty optional if buffering, but more messages are available.
        //   1) Wait until the message is deserialized.
        //   2) Skip until we are back on time.
        const std::optional<std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr>> optional_message = m_message_loader->next();

        if (optional_message.has_value()) {
            
            std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr> messgae = optional_message.value();
            m_next_message_time = messgae.first;
            m_next_message_data = messgae.second;

        }

        m_next_message_valid = optional_message.has_value();
       
    }
}

}
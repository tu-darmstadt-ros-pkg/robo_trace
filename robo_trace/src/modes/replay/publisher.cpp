// Base
#include "robo_trace/modes/replay/publisher.hpp"
// Std
#include <functional>
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"


namespace robo_trace {

 
MessagePublisher::MessagePublisher(ros_babel_fish::BabelFish& fish, ros::NodeHandle& node_handle, const PlayerOptions::ConstPtr& player_options, const MessageLoader::Ptr& loader, const DataContainer::Ptr& data) 
: m_message_loader(loader), m_player_options(player_options), m_next_message_valid(false) {

    /*
        Create a publisher for the topic.
    */

    const std::string topic = m_player_options->m_topic_prefix + data->getString("topic");
    const std::string message_type = data->getString("message_type");

    // TODO: Implement latching.
    m_publisher = fish.advertise(node_handle, message_type, topic, m_player_options->m_topic_queue_size, true);
    
}   

MessagePublisher::~MessagePublisher() {
    //
}

bool MessagePublisher::isBlocked() const {
    // We are only blocked if each and every buffer is empty and if there is more data available. 
    return !m_next_message_valid && 
           m_message_loader->getDeserializationBufferUtilization() == 0 &&
           m_message_loader->isBuffering(); 
}

bool MessagePublisher::isCompleted() const {
    return !m_next_message_valid && m_message_loader->isCompleted();
}
  
std::optional<double> MessagePublisher::getNextPublicationTime() {
    
    getNextMessage();
    //ROS_INFO_STREAM("Message valid: " << m_next_message_valid << " - " << m_publisher.getTopic() 
    //                 << " - BU: " << m_message_loader->getDeserializationBufferUtilization() 
    //               << " - B: " << m_message_loader->isBuffering()
    //               << " - C: " << isCompleted());
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

    getNextMessage();

}
  
void MessagePublisher::publish() {

    // Fetch the next message if we haven't already (should actually never happen, because publish is invoked on the correct time only).
    getNextMessage();

    // No more messages available 
    if (!m_next_message_valid) { 
        return;
    }
    
    ROS_INFO_STREAM("Sending message to " << m_publisher.getTopic());

    m_publisher.publish(m_next_message_data);
    m_next_message_valid = false;
    
    // Prepare the next message.
    getNextMessage();

}
    
void MessagePublisher::getNextMessage() {

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
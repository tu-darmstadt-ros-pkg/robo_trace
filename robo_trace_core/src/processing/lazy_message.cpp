// Base
#include "robo_trace_core/processing/lazy_message.hpp"
// Ros
#include "ros/console.h"
// Babel Fish
#include <ros_babel_fish/messages/compound_message.h>


namespace robo_trace {

LazyMessage::LazyMessage(ros_babel_fish::BabelFish& fish, const ros_babel_fish::BabelFishMessage::ConstPtr description)
 : Message(description, Message::Status::StreamLeading), m_fish(fish) {
    //
}

LazyMessage::~LazyMessage() {
    //
}


size_t LazyMessage::getLength() {

    if (m_status == Message::Status::MessageLeading) {
        synchronize();
    }

     // The case, where we still have the original stream.
    if (m_stream_data == nullptr) {
        ROS_INFO_STREAM(" - Returning default buffer size");
        return m_description->size();
    } else {
        ROS_INFO_STREAM(" - Returning our buffer size");
        return m_stream_length;
    }   

}

const uint8_t* const LazyMessage::getStream() {

    if (m_status == Message::Status::MessageLeading) {
        synchronize();
    }

    // The case, where we still have the original stream.
    if (m_stream_data == nullptr) {
        ROS_INFO_STREAM(" - Returning default buffer");
        return m_description->buffer();
    } else {
        ROS_INFO_STREAM(" - Returning our buffer");
        return m_stream_data.get();
    }

}

void LazyMessage::setStream(std::unique_ptr<uint8_t[]> stream, const size_t length) {
    // Assuming stream and decoded message are synched.
    m_status = Message::Status::StreamLeading;
    // Transfer ownership.
    m_stream_data = std::move(stream);
    m_stream_length = length;
}

bool LazyMessage::isDecodable() {
    // For now we say that the stream is decoadble as long as there
    // have been no modifications to the stream. This constrain may
    // be relaxed. As soon as the stream does not represent the message
    // template anymore, decoding is pointless however (unless we 
    // create templates at runtime). 
    return m_stream_data == nullptr;
}

const ros_babel_fish::Message::Ptr& LazyMessage::getDecoded() {

    if (m_status == Message::Status::StreamLeading) {
        synchronize();
    }
    
    // Assuming the caller will modify the message.
    m_status = Message::Status::MessageLeading;
    return m_message;

}

void LazyMessage::synchronize() {
    ROS_INFO_STREAM(" - Synchronizing lazy msg");
    switch(m_status) {
        
        // Construct message from stream
        case Message::Status::StreamLeading: {
            
            if (!isDecodable()) {
                // TODO: Except
            }

            // TODO: If we were to allow for dynamic type change this would need to fetch from somehwere else.
            const ros_babel_fish::MessageDescription::ConstPtr& message_description = m_fish.descriptionProvider()->getMessageDescription(*m_description);

            if (message_description == nullptr) {
                throw ros_babel_fish::BabelFishException("BabelFish failed to get message description for received message of type: " + m_description->dataType());
            }

            const ros_babel_fish::MessageTemplate::ConstPtr& msg_template = message_description->message_template;

            size_t bytes_read = 0;
            // :(
            ros_babel_fish::Message::Ptr translated(ros_babel_fish::CompoundMessage::fromStream(msg_template, getStream(), getLength(), bytes_read));
            
            if (bytes_read != m_description->size()) {
                throw ros_babel_fish::BabelFishException("Translated message did not consume all message bytes!");
            }

            m_message = translated;

            break;
        }
        // Construct stream from message
        case Message::Status::MessageLeading: {   

            m_stream_length = m_message->_sizeInBytes();  
            m_stream_data = std::unique_ptr<uint8_t[]>(new uint8_t[m_stream_length]);
            // :(
            m_message->writeToStream(m_stream_data.get());

            break;
        }  
        // If synchronized. 
        default:
            break;
        
    }
}

}
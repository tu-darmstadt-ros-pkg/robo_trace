// Base
#include "robo_trace_plugin_interface/processing/message.hpp"


namespace robo_trace {

Message::Message(const ros_babel_fish::BabelFishMessage::ConstPtr ingress_message) 
: m_ingress_message(ingress_message) {
    //
}

Message::~Message() = default;

size_t Message::getStreamLength() const {
    if (m_serialized_message.isEmpty()) {
        return getIngressStreamLength();
    } else {
        return getSerializedStreamLength();
    }
} 

const uint8_t* const Message::getStreamData() const {
    if (m_serialized_message.isEmpty()) {
        return getIngressStreamData();
    } else {
        return getSerializedStreamData();
    }
}

size_t Message::getIngressStreamLength() const {
    return m_ingress_message->size();
}

const uint8_t* const Message::getIngressStreamData() const {
    return m_ingress_message->buffer();
}

const ros_babel_fish::BabelFishMessage::ConstPtr& Message::getIngress() const {
    return m_ingress_message;
}

size_t Message::getSerializedStreamLength() const {
    return m_serialized_message.objsize();
}

const uint8_t* const Message::getSerializedStreamData() const {
    return reinterpret_cast<const uint8_t*>(m_serialized_message.objdata());
}

const mongo::BSONObj& Message::getSerialized() const {
    return m_serialized_message;
}

void Message::setSerialized(const mongo::BSONObj& serialized) {
    if (serialized.isOwned()) {
        m_serialized_message = serialized;
    } else {
        // Ensure there is no memory leak here.
        m_serialized_message = serialized.getOwned();
    }
}

}
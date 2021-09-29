// Base
#include "robo_trace/processing/context.hpp"

namespace robo_trace {

ProcessingContext::ProcessingContext()
: ProcessingContext(std::make_shared<DataContainer>()) {
    //
}

ProcessingContext::ProcessingContext(const DataContainer::Ptr& metadata)
: m_metadata(metadata),
  m_terminated(false) {
    //
}

ProcessingContext::~ProcessingContext() = default;

const DataContainer::Ptr& ProcessingContext::getMetadata() const {
    return m_metadata;
}

bool ProcessingContext::isTerminated() const {
    return m_terminated;
}

void ProcessingContext::setTerminated() {
    m_terminated = true;
}

bool ProcessingContext::isUnserialized() const {
    return m_unserialized_message.has_value();
}

const std::optional<ros_babel_fish::BabelFishMessage::ConstPtr>& ProcessingContext::getUnserializedMessage() const {
    return m_unserialized_message;
}

const std::optional<const uint8_t* const> ProcessingContext::getUnserializedMessage(size_t& length) const {
    if (m_unserialized_message) {

        ros_babel_fish::BabelFishMessage::ConstPtr message = m_unserialized_message.value();
        length = message->size();
        
        return reinterpret_cast<const uint8_t*>(message->buffer());

    } else {
        length = 0;
        return {};
    }
}

void ProcessingContext::setUnserializedMessage(const ros_babel_fish::BabelFishMessage::ConstPtr& ingress) {
    m_unserialized_message = ingress;
}

bool ProcessingContext::isSerialized() const {
    return m_serialized_message.has_value();
}

const std::optional<mongo::BSONObj>& ProcessingContext::getSerializedMessage() const {
    return m_serialized_message;
}

const std::optional<const uint8_t* const> ProcessingContext::getSerializedMessage(size_t& length) const {
    if (m_serialized_message) {

        const mongo::BSONObj& metadata = m_serialized_message.value();
        length = metadata.objsize();
        
        return reinterpret_cast<const uint8_t*>(metadata.objdata());

    } else {
        length = 0;
        return {};
    }
}

void ProcessingContext::setSerializedMessage(const mongo::BSONObj& serialized) {
    m_serialized_message = serialized;
}

}
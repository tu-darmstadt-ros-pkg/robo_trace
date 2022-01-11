// Base
#include "robo_trace/processing/context.hpp"


namespace robo_trace::processing {

Context::Context()
: Context(std::make_shared<robo_trace::store::Container>()) {
    //
}

Context::Context(const robo_trace::store::Container::Ptr& metadata)
: m_metadata(metadata),
  m_terminated(false) {
    //
}

Context::~Context() = default;

const robo_trace::store::Container::Ptr& Context::getMetadata() const {
    return m_metadata;
}

bool Context::isTerminated() const {
    return m_terminated;
}

void Context::setTerminated() {
    m_terminated = true;
}

bool Context::isUnserialized() const {
    return m_unserialized_message.has_value();
}

const std::optional<ros_babel_fish::BabelFishMessage::ConstPtr>& Context::getUnserializedMessage() const {
    return m_unserialized_message;
}

const std::optional<const uint8_t*> Context::getUnserializedMessage(size_t& length) const {
    if (m_unserialized_message) {

        ros_babel_fish::BabelFishMessage::ConstPtr message = m_unserialized_message.value();
        length = message->size();
        
        return reinterpret_cast<const uint8_t*>(message->buffer());

    } else {
        length = 0;
        return {};
    }
}

void Context::setUnserializedMessage(const ros_babel_fish::BabelFishMessage::ConstPtr& ingress) {
    m_unserialized_message = ingress;
}

bool Context::isSerialized() const {
    return m_serialized_message.has_value();
}

const std::optional<bsoncxx::document::view>& Context::getSerializedMessage() const {
    return m_serialized_message;
}

const std::optional<const uint8_t* const> Context::getSerializedMessage(size_t& length) const {
    if (m_serialized_message) {

        const bsoncxx::document::view& metadata = m_serialized_message.value();
        length = metadata.length();

        return metadata.data();

    } else {
        length = 0;
        return {};
    }
}

void Context::setSerializedMessage(const bsoncxx::document::value& serialized) {
    m_serialized_owning = serialized;
    m_serialized_message = m_serialized_owning.value().view();
}

void Context::setSerializedMessage(const bsoncxx::document::view serialized) {
    m_serialized_owning = {};
    m_serialized_message = serialized;
}

}
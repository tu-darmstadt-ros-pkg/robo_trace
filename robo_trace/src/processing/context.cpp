// Base
#include "robo_trace/processing/context.hpp"


namespace robo_trace::processing {

Context::Context()
: Context(std::make_shared<robo_trace::store::Container>()) {
    //
}

Context::Context(const robo_trace::store::Container::Ptr& metadata)
: m_metadata(metadata),
  m_persistor({}),
  m_terminated(false) {
    //
}

Context::Context(const robo_trace::store::Container::Ptr& metadata, const robo_trace::store::Persistor::Ptr& persistor)
: m_metadata(metadata),
  m_persistor(persistor),
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

bool Context::getHasPersistor() const {
    return m_persistor.has_value();
}

const std::optional<robo_trace::store::Persistor::Ptr>& Context::getPersistor() const {
    return m_persistor;
}

bool Context::getHasRosMessage() const {
    return m_unserialized_message.has_value();
}

const std::optional<ros_babel_fish::BabelFishMessage::ConstPtr>& Context::getRosMessage() const {
    return m_unserialized_message;
}

const std::optional<const uint8_t*> Context::getRosMessageStream(size_t& length) const {
    if (m_unserialized_message) {

        ros_babel_fish::BabelFishMessage::ConstPtr message = m_unserialized_message.value();
        length = message->size();
        
        return reinterpret_cast<const uint8_t*>(message->buffer());

    } else {
        length = 0;
        return {};
    }
}

void Context::setRosMessage(const ros_babel_fish::BabelFishMessage::ConstPtr& ingress) {
    m_unserialized_message = ingress;
}

bool Context::getHasBsonMessage() const {
    return m_bson_message.has_value();
}

const std::optional<bsoncxx::document::view>& Context::getBsonMessage() const {
    return m_bson_message;
}

const std::optional<const uint8_t* const> Context::getBsonMessageStream(size_t& length) const {
    if (m_bson_message) {

        const bsoncxx::document::view& metadata = m_bson_message.value();
        length = metadata.length();

        return metadata.data();

    } else {
        length = 0;
        return {};
    }
}

void Context::setBsonMessage(const bsoncxx::document::value& serialized) {
    m_bson_owning = serialized;
    m_bson_message = m_bson_owning.value().view();
}

void Context::setBsonMessage(const bsoncxx::document::view serialized) {
    m_bson_owning = {};
    m_bson_message = serialized;
}

}
/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/context.hpp"


namespace robo_trace::processing {

Context::Context(const robo_trace::store::StreamHandler::Ptr stream)
: Context(std::make_shared<robo_trace::store::Container>(), stream) {
    //
}

Context::Context(const robo_trace::store::Container::Ptr& metadata, const robo_trace::store::StreamHandler::Ptr stream)
: m_metadata(metadata),
  m_persistor({}),
  m_stream_handler(stream),
  m_terminated(false) {
    //
}

Context::Context(const robo_trace::store::Container::Ptr& metadata, const robo_trace::store::Persistor::Ptr& persistor, const robo_trace::store::StreamHandler::Ptr stream)
: m_metadata(metadata),
  m_persistor(persistor),
  m_stream_handler(stream),
  m_terminated(false) {
    //
}

Context::~Context() = default;

const robo_trace::store::Container::Ptr& Context::getMetadata() const {
    return m_metadata;
}

const robo_trace::store::StreamHandler::Ptr Context::getStreamHandler() const {
    return m_stream_handler;
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
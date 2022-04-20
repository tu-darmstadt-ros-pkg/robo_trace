/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/modules/marshalling/forward.hpp"
// MongoCXX
#include <bsoncxx/builder/basic/kvp.hpp>
// BabelFish
#include <ros_babel_fish/message_description.h>
// Project
#include "robo_trace/processing/translation/translator.hpp"


namespace robo_trace::processing {

BasicMarshallingForwardProcessor::BasicMarshallingForwardProcessor(const ros_babel_fish::MessageTemplate::ConstPtr message_template)
: m_message_template(message_template) {
    //
}

BasicMarshallingForwardProcessor::~BasicMarshallingForwardProcessor() {
    //
}

Mode BasicMarshallingForwardProcessor::getMode() const {
    return Mode::CAPTURE;
}

void BasicMarshallingForwardProcessor::process(const Context::Ptr& context) {
    
    const std::optional<ros_babel_fish::BabelFishMessage::ConstPtr>& o_message = context->getRosMessage();

    if (!o_message) {
        throw std::runtime_error("No message to be serialized provided.");
    }

    const ros_babel_fish::BabelFishMessage::ConstPtr& msg = o_message.value();
    const uint8_t* stream = msg->buffer();

    bsoncxx::builder::basic::document builder{};
    Translation::serialize(m_message_template, builder, &stream);
    
    context->setBsonMessage(builder.extract());
}

}
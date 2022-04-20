/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once
// MongoCXX
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/builder/basic/document.hpp>
// BabelFish 
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/messages/compound_message.h>

namespace robo_trace::processing {

class Translation {

private:

    /*
     *
     */
    Translation();

public:

    /**
     * 
     */
    static void upload(bsoncxx::builder::basic::sub_document& builder, const std::string& name, const uint8_t** stream, const size_t length);

    /**
     * 
     */
    static void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, bsoncxx::builder::basic::sub_document& builder, const uint8_t** stream);
    
    /** 
     * 
     */
    static void advance(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const uint8_t** stream);

    /**
     * 
     */
    static void deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const bsoncxx::document::view& serialized, ros_babel_fish::CompoundMessage& deserialized);

};

}
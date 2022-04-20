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
// BabelFish
#include <ros_babel_fish/message_description.h>
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
#include <ros_babel_fish/messages/compound_message.h>
// Project
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::processing {

class BasicMarshallingBackwardProcessor final : public Processor { 

public:

    /**
     * 
     */
    BasicMarshallingBackwardProcessor(const ros_babel_fish::MessageDescription::ConstPtr& message_description);
  
    /**
     * 
     */
    virtual ~BasicMarshallingBackwardProcessor();

    /**
     *
     */
    virtual Mode getMode() const final override;
   
    /**
     * 
     */
    virtual void process(const Context::Ptr& context) final override;

private:

    /** */
    ros_babel_fish::MessageDescription::ConstPtr m_message_description;

};

}
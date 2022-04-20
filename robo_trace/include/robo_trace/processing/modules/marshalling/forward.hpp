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
#include <bsoncxx/builder/basic/document.hpp>
// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::processing {

class BasicMarshallingForwardProcessor final : public Processor { 

public:

    /**
     * 
     */
    BasicMarshallingForwardProcessor(const ros_babel_fish::MessageTemplate::ConstPtr message_template);
  
    /**
     * 
     */
    virtual ~BasicMarshallingForwardProcessor();
    
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
    ros_babel_fish::MessageTemplate::ConstPtr m_message_template;

};

}
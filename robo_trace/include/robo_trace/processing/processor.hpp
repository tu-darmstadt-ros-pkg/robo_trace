/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once

// Std
#include <string>
#include <memory>
// Project
#include "robo_trace/processing/mode.hpp"
#include "robo_trace/processing/context.hpp"


namespace robo_trace::processing {

class Processor {

public:

    typedef std::shared_ptr<Processor> Ptr;
    typedef std::shared_ptr<const Processor> ConstPtr;

public:

    /**
     * Constructs a new processing module.
     */
    Processor();
  
    /**
     * Destructs this instance.
     */
    virtual ~Processor();

    /**
     * 
     */
    virtual Mode getMode() const = 0;

    /**
     * Implements the processing step realized by this module on
     * the provided context.
     * 
     * @param ctx the message context to process.
     */
    virtual void process(const Context::Ptr& ctx) = 0;

};

}
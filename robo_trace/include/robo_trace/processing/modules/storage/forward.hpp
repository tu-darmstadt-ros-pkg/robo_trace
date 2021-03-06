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
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/storage/persistor.hpp"
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::processing {

class StorageForwardProcessor final : public Processor { 

public:

    /**
     * 
     */
    StorageForwardProcessor(const robo_trace::store::Persistor::Ptr& persistor, const robo_trace::store::Container::Ptr& metadata);
  
    /**
     * 
     */
    virtual ~StorageForwardProcessor();
    
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
    const robo_trace::store::Persistor::Ptr m_persistor;

};

}
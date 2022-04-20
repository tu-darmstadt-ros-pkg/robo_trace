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
#include <malloc.h>
#include <memory>
#include "robo_trace_core/config.h"
// MongoDB
#include <mongo/db/json.h>


namespace robo_trace {

class WrappedBSON : public  mongo::BSONObj {

public:

    /**
     * 
     */
    WrappedBSON();

    /**
     * 
     */
    WrappedBSON(const WrappedBSON& other);

    /**
     * 
     */
    WrappedBSON(const BSONObj& other);

    /**
     * 
     */
    WrappedBSON(const std::string& json);

protected:

    /**
     * 
     */
    void update();

protected:

    /** */
    std::shared_ptr<mongo::BSONObjBuilder> m_builder;

};

}
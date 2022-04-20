/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace_core/utils/wrapped_bson.hpp"


namespace robo_trace {

using mongo::BSONObj;
using mongo::BSONObjBuilder;

WrappedBSON::WrappedBSON() 
: BSONObj(), m_builder(new BSONObjBuilder()) {
    //
}

WrappedBSON::WrappedBSON(const WrappedBSON& other) 
: BSONObj(), m_builder(other.m_builder) {
    update();
}

WrappedBSON::WrappedBSON(const BSONObj& other) 
: BSONObj(), m_builder(new BSONObjBuilder()) {
    m_builder->appendElements(other);
    update();
}

WrappedBSON::WrappedBSON(const std::string& json) 
: BSONObj(), m_builder(new BSONObjBuilder()) {
    m_builder->appendElements(mongo::fromjson(json.c_str()));
    update();
}

void WrappedBSON::update() {
    BSONObj::operator=(m_builder->asTempObj());
}

}
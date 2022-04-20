/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once
// Ros
#include "ros/ros.h"
// Nodelet
#include "nodelet/nodelet.h"


namespace robo_trace {

class RoboTraceNodelet final : public nodelet::Nodelet {

public:

    /**
     * 
     */
    RoboTraceNodelet();

    /**
     * 
     */
    ~RoboTraceNodelet();

private:

    /**
     * 
     */
    virtual void onInit() final override; 

};

}

PLUGINLIB_EXPORT_CLASS(robo_trace::RoboTraceNodelet, nodelet::Nodelet);
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
#include <vector>
// Boost
#include "boost/program_options.hpp"
// Ros
#include <ros/ros.h> 


namespace po = boost::program_options;

namespace robo_trace::util {

class Options {

public:

    typedef std::shared_ptr<Options> Ptr;
    typedef std::shared_ptr<const Options> ConstPtr;

public:

    static void load(const std::vector<Options::Ptr>& containers, ros::NodeHandle& node_handle, int argc, char** argv);

public:

    /**
     *
     */
    Options();

    /**
     *
     */
    virtual ~Options(); 

public:

    /**
     *
     */
    virtual void load(const ros::NodeHandle& node_handle) = 0;

    /**
     *
     */
    virtual void setup(po::options_description& description) = 0;

    /**
     *
     */
    virtual void load(po::variables_map& options) = 0;

    /**
     *
     */
    virtual void validate();

};

}
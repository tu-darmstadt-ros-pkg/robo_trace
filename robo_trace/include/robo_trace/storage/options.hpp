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
// ROS
#include <ros/ros.h>
// Project
#include "robo_trace/util/options.hpp"


namespace robo_trace::store {

class Options final : public robo_trace::util::Options {

public:

    typedef std::shared_ptr<Options> Ptr;
    typedef std::shared_ptr<const Options> ConstPtr;

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
    virtual void load(const ros::NodeHandle& node_handle) final override;

    /**
     *
     */
    virtual void setup(po::options_description& description) final override;

    /**
     *
     */
    virtual void load(po::variables_map& options) final override;
    
    /**
     *
     */
    virtual void validate() final override;

public:

    /** */
    int m_host_port;
    /** */
    std::string m_host_name;

    /** */
    std::string m_database_name;
    /** */
    std::string m_collection_name_summary;

    /** */
    int m_connection_pool_size_min;
    /** */
    int m_connection_pool_size_max;
    /** */
    int m_connection_timeout_ms;

};

}
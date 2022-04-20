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
#include <memory>
#include <string>
#include <vector>
// Ros
#include <ros/ros.h>
// Project
#include "robo_trace/processing/descriptor.hpp"


namespace robo_trace::processing {

class Plugin {

public:

    typedef std::shared_ptr<Plugin> Ptr;
    typedef std::shared_ptr<const Plugin> ConstPtr;

public:

    /**
     * 
     */
    Plugin(const std::string& name);

    /**
     * 
     */
    virtual ~Plugin();

    /**
     * 
     */
    const std::string& getName() const;

    /**
     * 
     */
    const std::vector<robo_trace::processing::Descriptor::Ptr>& getModules() const;

    /**
     * 
     */
    ros::NodeHandle& getSystemNodeHandle();

    /**
     * 
     */
    ros::NodeHandle& getPluginNodeHandle();

    /**
     * 
     */
    ros::NodeHandle& getStageNodeHandle();

    /**
     * 
     */
    void initialize(ros::NodeHandle& system_node_handle, ros::NodeHandle& plugin_node_handle, ros::NodeHandle& stage_node_handle);   

protected:

    /**
     * 
     */
    virtual std::vector<Descriptor::Ptr> setup() = 0;

protected:

    /** */
    const std::string m_name;

    /** */
    ros::NodeHandle m_system_node_handle;
    /** */
    ros::NodeHandle m_plugin_node_handle;
    /** */
    ros::NodeHandle m_stage_node_handle;

    /** */
    std::vector<Descriptor::Ptr> m_modules;

};

}